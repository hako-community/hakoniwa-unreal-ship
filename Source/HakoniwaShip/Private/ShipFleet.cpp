// ShipFleet.cpp

#include "ShipFleet.h"

#include "Camera/CameraComponent.h"
#include "Dom/JsonObject.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HakoniwaClientInterface.h"
#include "HakoniwaShmClient.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "PduManager.h"
#include "DynamicRHI.h"
#include "RenderTimer.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "ShipActor.h"
#include "ShipFrame.h"
#include "ShipOcean.h"
#include "EngineUtils.h"
#include "UnrealClient.h"

#include "pdu_convertor.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
// この版の hakoniwa-pdu-registry は文字列の可変長配列の変換を ros 名でだけ持つ（hakoniwa-unreal-rover の RoverAvatar と同じ読み替え）
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "sensor_msgs/pdu_cpptype_conv_JointState.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray
#include "hako_msgs/pdu_cpptype_conv_Disturbance.hpp"

DEFINE_LOG_CATEGORY_STATIC(LogShipFleet, Log, All);

namespace
{
	FVector JsonVec3(const TSharedPtr<FJsonObject>& O, const FString& Key)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		if (!O || !O->TryGetArrayField(Key, A) || A->Num() != 3)
		{
			return FVector::ZeroVector;
		}
		return FVector((*A)[0]->AsNumber(), (*A)[1]->AsNumber(), (*A)[2]->AsNumber());
	}

	/** 右手系の回転 R で v を回す（列ベクトルの約束） */
	FVector Rotate(const FMatrix& R, const FVector& V)
	{
		return FVector(R.M[0][0] * V.X + R.M[0][1] * V.Y + R.M[0][2] * V.Z,
			R.M[1][0] * V.X + R.M[1][1] * V.Y + R.M[1][2] * V.Z,
			R.M[2][0] * V.X + R.M[2][1] * V.Y + R.M[2][2] * V.Z);
	}

	/** Unreal の向き → 右手系の向き（S = diag(1, −1, 1)） */
	FVector DirToLocal(const FVector& U) { return FVector(U.X, -U.Y, U.Z); }

	double WrapPi(double A) { return FMath::Atan2(FMath::Sin(A), FMath::Cos(A)); }

	/** fleet_ue.json の波浪（wave.preview / plant_check.wave）→ WaveParams。潮位は物理プラグインと同じく 0 */
	bool JsonWave(const TSharedPtr<FJsonObject>& O, ShipWave::FParams& W)
	{
		if (!O)
		{
			return false;
		}
		W = ShipWave::FParams();
		W.hs_m = O->GetNumberField(TEXT("hs_m"));
		W.tp_s = O->GetNumberField(TEXT("tp_s"));
		W.wave_dir_rad = O->GetNumberField(TEXT("wave_dir_rad"));
		W.gamma = O->GetNumberField(TEXT("gamma"));
		W.seed = static_cast<uint32_t>(O->GetNumberField(TEXT("seed")));
		W.water_depth_m = O->GetNumberField(TEXT("water_depth_m"));
		W.tide_level_m = 0.0;
		return true;
	}
}

double AShipFleet::FHeaveAcc::Corr() const
{
	const double Vx = N * Sxx - Sx * Sx;
	const double Vy = N * Syy - Sy * Sy;
	return (N > 1 && Vx > 0.0 && Vy > 0.0) ? (N * Sxy - Sx * Sy) / FMath::Sqrt(Vx * Vy) : 0.0;
}

AShipFleet::AShipFleet()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Fleet"));
	ChaseCam = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCam"));
	ChaseCam->SetupAttachment(RootComponent);
	ChaseCam->SetUsingAbsoluteLocation(true);
	ChaseCam->SetUsingAbsoluteRotation(true);
	ChaseCam->SetFieldOfView(60.0f);
}

bool AShipFleet::LoadFleet()
{
	const FString Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / FleetConfig);
	FString Text;
	TSharedPtr<FJsonObject> Root;
	if (!FFileHelper::LoadFileToString(Text, *Path) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) || !Root)
	{
		UE_LOG(LogShipFleet, Error, TEXT("fleet config not readable: %s"), *Path);
		return false;
	}
	AssetName = Root->GetStringField(TEXT("asset_name"));
	const TSharedPtr<FJsonObject> Clock = Root->GetObjectField(TEXT("physics_clock"));
	ClockRobot = Clock->GetStringField(TEXT("robot"));
	ClockPdu = Clock->GetStringField(TEXT("pdu"));
	int32 MaxSize = 0;
	for (const TSharedPtr<FJsonValue>& V : Root->GetArrayField(TEXT("pdus")))
	{
		const TSharedPtr<FJsonObject> P = V->AsObject();
		FShipPduRead R;
		R.Robot = P->GetStringField(TEXT("robot"));
		R.Pdu = P->GetStringField(TEXT("pdu"));
		R.Type = P->GetStringField(TEXT("type"));
		R.Channel = P->GetIntegerField(TEXT("channel_id"));
		R.Size = P->GetIntegerField(TEXT("pdu_size"));
		MaxSize = FMath::Max(MaxSize, R.Size);
		PduTable.Add(R);
	}
	Buf.SetNumZeroed(MaxSize);
	for (const TSharedPtr<FJsonValue>& V : Root->GetArrayField(TEXT("ships")))
	{
		const TSharedPtr<FJsonObject> S = V->AsObject();
		AShipActor* Ship = GetWorld()->SpawnActor<AShipActor>();
		if (!Ship || !Ship->BuildFromSpec(S))
		{
			return false;
		}
		const TSharedPtr<FJsonObject> Preview = S->GetObjectField(TEXT("preview_pose"));
		Ship->ApplyPose(JsonVec3(Preview, TEXT("linear")), JsonVec3(Preview, TEXT("angular")));
		Ships.Add(Ship);
		FShipSync Sh;
		Sh.Actor = Ship;
		Sh.PosPdu = S->GetStringField(TEXT("pos_pdu"));
		S->TryGetStringField(TEXT("joint_states_pdu"), Sh.JointPdu);
		Sync.Add(Sh);
	}
	// 海面（Step 3）
	const TSharedPtr<FJsonObject> Wave = Root->GetObjectField(TEXT("wave"));
	const TSharedPtr<FJsonObject> Dist = Wave->GetObjectField(TEXT("disturbance"));
	DistRobot = Dist->GetStringField(TEXT("robot"));
	DistPdu = Dist->GetStringField(TEXT("pdu"));
	WaveComponents = Wave->GetIntegerField(TEXT("num_components"));
	WaveActiveHs = Wave->GetNumberField(TEXT("active_hs_m"));
	bHasPreviewWave = JsonWave(Wave->GetObjectField(TEXT("preview")), PreviewWave);
	const TSharedPtr<FJsonObject> Check = Root->GetObjectField(TEXT("plant_check"));
	SettleSec = Check->GetNumberField(TEXT("settle_s"));
	HeaveCorrMin = Check->GetNumberField(TEXT("heave_corr_min"));
	WaterlineRmsMax = Check->GetNumberField(TEXT("waterline_rms_max_m"));
	bHasExpectedWave = JsonWave(Check->GetObjectField(TEXT("wave")), ExpectedWave);
	const double Tp = ExpectedWave.tp_s;
	HeaveLags = { 0.0, 0.5, 1.0, 1.5, 0.5 * Tp, -0.5 * Tp };
	const TSharedPtr<FJsonObject> CheckShips = Check->GetObjectField(TEXT("ships"));
	for (FShipSync& S : Sync)
	{
		const TSharedPtr<FJsonObject>* C = nullptr;
		if (CheckShips->TryGetObjectField(S.Actor->Robot, C))
		{
			S.LppM = (*C)->GetNumberField(TEXT("lpp_m"));
			S.BeamM = (*C)->GetNumberField(TEXT("beam_m"));
			S.Heave.SetNum(HeaveLags.Num());
		}
	}
	UE_LOG(LogShipFleet, Log, TEXT("fleet %s: %d ships, %d PDUs, clock %s/%s, asset %s"), *Path, Ships.Num(), PduTable.Num(), *ClockRobot, *ClockPdu, *AssetName);
	return Ships.Num() > 0;
}

void AShipFleet::BeginPlay()
{
	Super::BeginPlay();
	const TCHAR* Cmd = FCommandLine::Get();
	bSync = FParse::Param(Cmd, TEXT("ShipSync"));
	bHakoStart = FParse::Param(Cmd, TEXT("ShipHakoStart"));
	float Sec = 0.0f;
	if (FParse::Value(Cmd, TEXT("-ShipSyncCheck="), Sec) && Sec > 0.0f)
	{
		CheckSec = Sec;
	}
	float Interval = 0.0f;
	if (FParse::Value(Cmd, TEXT("-ShipSampleInterval="), Interval) && Interval > 0.0f)
	{
		SampleInterval = Interval;
	}
	FParse::Value(Cmd, TEXT("-ShipSamples="), SamplesPath);
	FParse::Value(Cmd, TEXT("-ShipShot="), ShotPath);
	FParse::Value(Cmd, TEXT("-ShipFrameLog="), FrameLogPath);
	bChase = FParse::Param(Cmd, TEXT("ShipChase")) || !ShotPath.IsEmpty();
	if (!LoadFleet())
	{
		if (CheckSec > 0.0)
		{
			UE_LOG(LogShipFleet, Error, TEXT("HAKONIWA_UNREAL_SHIP_SYNC_FAILED fleet config"));
			FPlatformMisc::RequestExitWithStatus(false, 1);
		}
		return;
	}
	UE_LOG(LogShipFleet, Log, TEXT("sync=%d hako_start=%d check=%.1f s samples=%s HAKO_CONFIG_PATH=%s"),
		bSync, bHakoStart, CheckSec, *SamplesPath, *FPlatformMisc::GetEnvironmentVariable(TEXT("HAKO_CONFIG_PATH")));
	// 海面：箱庭につないだときは Disturbance が届くまで静水（Godot と同じ）、つながないときは wave.preview。海面は船団の後に Tick する
	for (TActorIterator<AShipOcean> It(GetWorld()); It; ++It)
	{
		Ocean = *It;
		break;
	}
	if (Ocean)
	{
		Ocean->Configure(WaveComponents, WaveActiveHs);
		Ocean->AddTickPrerequisiteActor(this);
		if (!bSync && bHasPreviewWave)
		{
			Ocean->SetWaveParams(PreviewWave);
		}
	}
	else
	{
		UE_LOG(LogShipFleet, Warning, TEXT("no AShipOcean in the map (run scripts\\windows\\ship_scene.ps1 -Build)"));
	}
	if (bSync && !FindPduManager())
	{
		// rover の ARoverAvatar と同じく -ShipSync のときだけ箱庭のアセットを置く。BeginPlay で登録し、OnHakoInitialize で PDU を宣言する
		FActorSpawnParameters P;
		P.bDeferConstruction = true;
		AHakoniwaShmClient* Client = GetWorld()->SpawnActor<AHakoniwaShmClient>(AHakoniwaShmClient::StaticClass(), FTransform::Identity, P);
		if (Client)
		{
			Client->ConfigPath = WritePduConfig();
			Client->AssetName = AssetName;
			Client->FinishSpawning(FTransform::Identity);
		}
	}
}

FString AShipFleet::WritePduConfig() const
{
	// PduManager の PDU の表（pdu_config_ship.json と同じ形。読む PDU だけ）。PduManager は Content からの相対で読むので、
	// Saved に書いて Content からの相対の道を返す（rover と同じ）
	TArray<FString> Robots;
	for (const FShipPduRead& R : PduTable)
	{
		Robots.AddUnique(R.Robot);
	}
	TArray<FString> RobotJson;
	for (const FString& Robot : Robots)
	{
		TArray<FString> Readers;
		for (const FShipPduRead& R : PduTable)
		{
			if (R.Robot == Robot)
			{
				Readers.Add(FString::Printf(TEXT("{\"type\":\"%s\",\"org_name\":\"%s\",\"name\":\"%s_%s\",\"channel_id\":%d,\"pdu_size\":%d,\"write_cycle\":1,\"method_type\":\"SHM\"}"),
					*R.Type, *R.Pdu, *R.Robot, *R.Pdu, R.Channel, R.Size));
			}
		}
		RobotJson.Add(FString::Printf(TEXT("{\"name\":\"%s\",\"rpc_pdu_readers\":[],\"rpc_pdu_writers\":[],\"shm_pdu_readers\":[%s],\"shm_pdu_writers\":[]}"),
			*Robot, *FString::Join(Readers, TEXT(","))));
	}
	const FString Json = FString::Printf(TEXT("{\"robots\":[%s]}"), *FString::Join(RobotJson, TEXT(",")));
	const FString File = FPaths::ProjectSavedDir() / TEXT("HakoniwaShip/ship_pdu.json");
	FFileHelper::SaveStringToFile(Json, *File);
	FString Rel = FPaths::ConvertRelativePathToFull(File);
	FPaths::MakePathRelativeTo(Rel, *FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir()));
	UE_LOG(LogShipFleet, Log, TEXT("pdu config -> %s (from Content: %s)"), *File, *Rel);
	return Rel;
}

AActor* AShipFleet::FindClient() const
{
	TArray<AActor*> Actors;
	UGameplayStatics::GetAllActorsWithInterface(GetWorld(), UHakoniwaClientInterface::StaticClass(), Actors);
	return Actors.Num() > 0 ? Actors[0] : nullptr;
}

UPduManager* AShipFleet::FindPduManager() const
{
	AActor* Client = FindClient();
	return Client ? IHakoniwaClientInterface::Execute_GetPduManager(Client) : nullptr;
}

void AShipFleet::OnHakoInitialize_Implementation()
{
	UPduManager* Pdu = FindPduManager();
	if (!Pdu || !Pdu->IsServiceEnabled())
	{
		UE_LOG(LogShipFleet, Error, TEXT("OnHakoInitialize: PduManager is not ready"));
		return;
	}
	bool bAll = true;
	for (const FShipPduRead& R : PduTable)
	{
		const bool bOk = Pdu->DeclarePduForRead(R.Robot, R.Pdu);
		bAll &= bOk;
		UE_LOG(LogShipFleet, Log, TEXT("OnHakoInitialize: declare for read %s/%s (ch %d) = %d"), *R.Robot, *R.Pdu, R.Channel, bOk);
	}
	bDeclared = bAll;
}

bool AShipFleet::ReadPose(UPduManager* Pdu, FShipSync& S)
{
	if (!Pdu->ReadPduRawData(S.Actor->Robot, S.PosPdu, Buf) || Buf.Num() < 72)
	{
		return false;
	}
	HakoCpp_Twist T {};
	hako::pdu::PduConvertor<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> Conv;
	if (!Conv.pdu2cpp(reinterpret_cast<char*>(Buf.GetData()), T))
	{
		return false;
	}
	S.bNewPose = true;
	S.NewLinear = FVector(T.linear.x, T.linear.y, T.linear.z);
	S.NewAngular = FVector(T.angular.x, T.angular.y, T.angular.z);
	++S.PosFrames;
	return true;
}

bool AShipFleet::ReadJoints(UPduManager* Pdu, FShipSync& S, double& OutStamp)
{
	if (S.JointPdu.IsEmpty() || !Pdu->ReadPduRawData(S.Actor->Robot, S.JointPdu, Buf) || Buf.Num() == 0)
	{
		return false;
	}
	HakoCpp_JointState J {};
	hako::pdu::PduConvertor<HakoCpp_JointState, hako::pdu::msgs::sensor_msgs::JointState> Conv;
	if (!Conv.pdu2cpp(reinterpret_cast<char*>(Buf.GetData()), J) || J.name.empty())
	{
		return false;
	}
	S.JointNames.Reset();
	S.NewJoints.Reset();
	for (size_t i = 0; i < J.name.size(); ++i)
	{
		S.JointNames.Add(UTF8_TO_TCHAR(J.name[i].c_str()));
		S.NewJoints.Add(i < J.position.size() ? J.position[i] : 0.0);
	}
	S.bNewJoints = true;
	++S.JointFrames;
	OutStamp = J.header.stamp.sec + J.header.stamp.nanosec * 1.0e-9;
	return true;
}

void AShipFleet::PushSamples(FShipSync& S, double T)
{
	// 受信した値を物理時刻 T の標本として履歴に積む（同じ時刻なら置き換える）。古いものは捨てる
	if (S.bNewPose)
	{
		S.bNewPose = false;
		if (S.Poses.Num() > 0 && S.Poses.Last().T >= T)
		{
			S.Poses.Pop();
		}
		S.Poses.Add({ T, S.NewLinear, S.NewAngular, {} });
	}
	if (S.bNewJoints)
	{
		S.bNewJoints = false;
		if (S.JointHist.Num() > 0 && S.JointHist.Last().T >= T)
		{
			S.JointHist.Pop();
		}
		S.JointHist.Add({ T, FVector::ZeroVector, FVector::ZeroVector, S.NewJoints });
	}
	constexpr int32 Keep = 32;
	if (S.Poses.Num() > Keep) { S.Poses.RemoveAt(0, S.Poses.Num() - Keep); }
	if (S.JointHist.Num() > Keep) { S.JointHist.RemoveAt(0, S.JointHist.Num() - Keep); }
}

int32 AShipFleet::Bracket(const TArray<FShipSample>& H, double T, double& OutAlpha)
{
	// H[i].T <= T <= H[i+1].T の i（範囲外は端に止める：OutAlpha = 0 で H[i] そのもの）
	OutAlpha = 0.0;
	if (H.Num() == 0) { return INDEX_NONE; }
	if (T <= H[0].T) { return 0; }
	for (int32 i = H.Num() - 2; i >= 0; --i)
	{
		if (H[i].T <= T)
		{
			const double Span = H[i + 1].T - H[i].T;
			OutAlpha = Span > 0.0 ? FMath::Clamp((T - H[i].T) / Span, 0.0, 1.0) : 1.0;
			return i;
		}
	}
	return H.Num() - 1;
}

void AShipFleet::ApplyAt(FShipSync& S, double T)
{
	// 表示時刻 T の姿勢：前後の受信値の線形補間（角度は差を ±π に折り返して補間）。物理は 20 ms ごとなので補間の誤差は見えない
	double Alpha = 0.0;
	const int32 i = Bracket(S.Poses, T, Alpha);
	if (i != INDEX_NONE)
	{
		const FShipSample& A = S.Poses[i];
		const FShipSample& B = S.Poses[FMath::Min(i + 1, S.Poses.Num() - 1)];
		const FVector Lin = FMath::Lerp(A.Lin, B.Lin, Alpha);
		const FVector Ang(A.Ang.X + Alpha * WrapPi(B.Ang.X - A.Ang.X), A.Ang.Y + Alpha * WrapPi(B.Ang.Y - A.Ang.Y), A.Ang.Z + Alpha * WrapPi(B.Ang.Z - A.Ang.Z));
		S.Actor->ApplyPose(Lin, Ang);
		S.LastLinear = Lin;
		S.LastAngular = Ang;

		// 検証 1：Unreal のアクタの位置と軸（エンジンが四元数から求めた前・右・上の向き）を、表示する値から右手系で直接求めた値と比べる。
		//   前 = R·x、右（Unreal の +Y）= R·(−y)、上 = R·z。四元数への変換（ShipFrame::RotationToUnreal）を通らない独立の計算
		const FMatrix R = ShipFrame::PduAngularToRotation(Ang.X, Ang.Y, Ang.Z);
		const double PosErr = (ShipFrame::UnrealToLocal(S.Actor->GetActorLocation()) - Lin).Size();
		const double AxisErr = FMath::Max3(
			(DirToLocal(S.Actor->GetActorForwardVector()) - Rotate(R, FVector(1, 0, 0))).Size(),
			(DirToLocal(S.Actor->GetActorRightVector()) - Rotate(R, FVector(0, -1, 0))).Size(),
			(DirToLocal(S.Actor->GetActorUpVector()) - Rotate(R, FVector(0, 0, 1))).Size());
		// 検証 2：関節の原点（船体の座標の定数）の世界の位置 = p + R·origin
		double PivotErr = 0.0;
		for (const FShipJoint& J : S.Actor->Joints)
		{
			const FVector Origin = ShipFrame::UnrealToLocal(J.Pivot->GetRelativeLocation());
			const FVector Want = Lin + Rotate(R, Origin);
			PivotErr = FMath::Max(PivotErr, (ShipFrame::UnrealToLocal(J.Pivot->GetComponentLocation()) - Want).Size());
		}
		S.MaxPosErr = FMath::Max3(S.MaxPosErr, PosErr, PivotErr);
		S.MaxAngErr = FMath::Max(S.MaxAngErr, AxisErr);   // 単位ベクトルの差 ≈ 角度 [rad]
	}
	const int32 j = Bracket(S.JointHist, T, Alpha);
	if (j != INDEX_NONE)
	{
		const TArray<double>& A = S.JointHist[j].Joints;
		const TArray<double>& B = S.JointHist[FMath::Min(j + 1, S.JointHist.Num() - 1)].Joints;
		TArray<double> Q;
		for (int32 k = 0; k < A.Num(); ++k)
		{
			Q.Add(k < B.Num() ? A[k] + Alpha * WrapPi(B[k] - A[k]) : A[k]);
		}
		S.Actor->ApplyJoints(S.JointNames, Q);
		// 検証 3：関節の部品の角度を読み戻す（軸まわりの回転角。2π の折り返しは除く）
		for (int32 k = 0; k < S.JointNames.Num() && k < Q.Num(); ++k)
		{
			double Back = 0.0;
			if (S.Actor->ReadBackJoint(S.JointNames[k], Back))
			{
				S.MaxJointErr = FMath::Max(S.MaxJointErr, FMath::Abs(WrapPi(Back - Q[k])));
			}
		}
	}
}

void AShipFleet::UpdateDisplayClock(double Dt)
{
	// 表示時刻：毎フレーム実時間で進め、最新の物理時刻 − DisplayDelay へゆっくり寄せる（±25 % の速さの調整）。
	// 物理は 50 Hz で、箱庭の許容遅延の範囲で間隔が揺れる。受信した時刻をそのまま使うと、描画（130〜200 FPS）が 2〜4 フレームごとに
	// 飛ぶのでカクつく。最新の受信より先は描かない（届かないときは止まる）。0.5 s 以上ずれたら合わせ直す
	const double Target = LastPhysicsTime - DisplayDelay;
	const double Prev = DisplayTime;
	if (DisplayTime < 0.0 || FMath::Abs(Target - DisplayTime) > 0.5)
	{
		if (DisplayTime >= 0.0) { ++ClockSnaps; }
		DisplayTime = Target;
	}
	else
	{
		DisplayTime += Dt * (1.0 + FMath::Clamp((Target - DisplayTime) * 2.0, -0.25, 0.25));
	}
	DisplayTime = FMath::Min(DisplayTime, LastPhysicsTime);
	++DisplayFrames;
	DisplayHolds += DisplayTime <= Prev ? 1 : 0;
	DisplayLagMax = FMath::Max(DisplayLagMax, LastPhysicsTime - DisplayTime);
}

void AShipFleet::ReadDisturbance(UPduManager* Pdu)
{
	// Godot の disturbance_sync.gd と同じ取り出し（ShipWave::ParseDisturbance）。同じ値なら海面は何もしない
	if (!Ocean || DistPdu.IsEmpty() || !Pdu->ReadPduRawData(DistRobot, DistPdu, Buf) || Buf.Num() == 0)
	{
		return;
	}
	HakoCpp_Disturbance D {};
	hako::pdu::PduConvertor<HakoCpp_Disturbance, hako::pdu::msgs::hako_msgs::Disturbance> Conv;
	if (!Conv.pdu2cpp(reinterpret_cast<char*>(Buf.GetData()), D))
	{
		return;
	}
	TArray<TArray<double>> Custom;
	for (const auto& C : D.d_user_custom)
	{
		Custom.Emplace(C.data.data(), static_cast<int32>(C.data.size()));
	}
	ShipWave::FParams W;
	if (ShipWave::ParseDisturbance(Custom, W))
	{
		++DistFrames;
		Ocean->SetWaveParams(W);
	}
}

void AShipFleet::SampleHeave()
{
	// Godot の plant_coupling_check.gd の _sample_heave と同じ：船体の水線面（長さ方向 10 点 × 幅方向 3 点）の η の平均と、
	// 船の上下位置（アクタから読み戻した z）を、海面の時刻から lag だけ戻した η と比べる
	constexpr int32 Stations = 10;
	const double TSea = Ocean->GetTime();
	for (FShipSync& S : Sync)
	{
		if (S.Heave.Num() == 0 || S.PosFrames == 0)
		{
			continue;
		}
		const FVector P = ShipFrame::UnrealToLocal(S.Actor->GetActorLocation());
		const double Yaw = S.LastAngular.Z;
		const FVector2D Fwd(FMath::Cos(Yaw), FMath::Sin(Yaw));
		const FVector2D Left(-FMath::Sin(Yaw), FMath::Cos(Yaw));
		for (int32 l = 0; l < HeaveLags.Num(); ++l)
		{
			const double T = TSea - HeaveLags[l];
			double Sum = 0.0;
			for (int32 s = 0; s < Stations; ++s)
			{
				const double Xi = ((s + 0.5) / Stations - 0.5) * S.LppM;
				for (const double Yi : { -0.5 * S.BeamM, 0.0, 0.5 * S.BeamM })
				{
					const FVector2D Q = FVector2D(P.X, P.Y) + Fwd * Xi + Left * Yi;
					Sum += Ocean->ElevationAt(Q.X, Q.Y, T);
				}
			}
			const double Eta = Sum / (Stations * 3);
			FHeaveAcc& A = S.Heave[l];
			A.Sx += P.Z; A.Sy += Eta; A.Sxx += P.Z * P.Z; A.Syy += Eta * Eta; A.Sxy += P.Z * Eta;
			A.Sd2 += (P.Z - Eta) * (P.Z - Eta);
			++A.N;
		}
	}
}

void AShipFleet::CheckSea(TArray<FString>& Problems)
{
	if (!Ocean)
	{
		Problems.Add(TEXT("no ocean"));
		return;
	}
	// 1. 波浪パラメータ（外乱パブリッシャが書いた値 = Godot の expected.json と同じ）
	const ShipWave::FParams& W = Ocean->GetWaveParams();
	UE_LOG(LogShipFleet, Log, TEXT("SHIP_SEA wave: disturbance frames %lld, active %d, %d components, Hs %.3f Tp %.3f dir %.3f gamma %.2f seed %u depth %.1f"),
		DistFrames, Ocean->IsWaveActive(), Ocean->ComponentCount(), W.hs_m, W.tp_s, W.wave_dir_rad, W.gamma, W.seed, W.water_depth_m);
	if (DistFrames == 0) { Problems.Add(TEXT("no maritime_disturbance")); }
	if (!Ocean->IsWaveActive() || Ocean->ComponentCount() != WaveComponents)
	{
		Problems.Add(FString::Printf(TEXT("ocean wave not active (%d components)"), Ocean->ComponentCount()));
	}
	if (bHasExpectedWave)
	{
		ShipWave::FParams E = ExpectedWave;
		E.num_components = W.num_components;
		if (!ShipWave::SameParams(E, W)) { Problems.Add(TEXT("wave parameters differ from plant_check.wave")); }
	}
	// 2. 海面の時刻 = 物理時刻（D19）
	UE_LOG(LogShipFleet, Log, TEXT("SHIP_SEA time: sea surface drawn at the ships' display time in %lld / %lld frames"), ClockFromPhysics, ClockFrames);
	if (ClockFrames == 0 || ClockFromPhysics < 0.99 * ClockFrames)
	{
		Problems.Add(FString::Printf(TEXT("sea surface not drawn at the ships' display time (%lld / %lld frames)"), ClockFromPhysics, ClockFrames));
	}
	// 3. 船体の上下動と海面（Godot と同じ閾値）
	for (const FShipSync& S : Sync)
	{
		if (S.Heave.Num() == 0)
		{
			continue;
		}
		FString Line = FString::Printf(TEXT("SHIP_SEA heave %-18s corr(z, eta(t-tau)):"), *S.Actor->Robot);
		double Best = -1.0, BestLag = 0.0;
		for (int32 l = 0; l < HeaveLags.Num(); ++l)
		{
			const double R = S.Heave[l].Corr();
			Line += FString::Printf(TEXT(" %+.1fs=%.3f"), HeaveLags[l], R);
			if (l < 4 && R > Best) { Best = R; BestLag = HeaveLags[l]; }
		}
		const FHeaveAcc& A0 = S.Heave[0];
		const double Rms = FMath::Sqrt(A0.Sd2 / FMath::Max<double>(1.0, A0.N));
		UE_LOG(LogShipFleet, Log, TEXT("%s | best %.1f s, waterline rms %.3f m, n=%lld"), *Line, BestLag, Rms, A0.N);
		if (A0.N == 0) { Problems.Add(FString::Printf(TEXT("%s: no heave samples"), *S.Actor->Robot)); continue; }
		if (Best < HeaveCorrMin) { Problems.Add(FString::Printf(TEXT("%s: heave does not follow the sea surface (best corr %.3f)"), *S.Actor->Robot, Best)); }
		if (S.Heave[4].Corr() >= 0.3 || S.Heave[5].Corr() >= 0.3) { Problems.Add(FString::Printf(TEXT("%s: heave correlates with the time-shifted sea surface"), *S.Actor->Robot)); }
		if (Rms > WaterlineRmsMax) { Problems.Add(FString::Printf(TEXT("%s: waterline rms %.3f m"), *S.Actor->Robot, Rms)); }
	}
	// 4. 描いた海面の高さ（AShipOcean の真上からの検証）
	UE_LOG(LogShipFleet, Log, TEXT("SHIP_SEA surface: %d checks, %lld points, misses %lld, max err %.2e m, max |eta| %.2f m"),
		Ocean->SurfaceChecks, Ocean->SurfacePoints, Ocean->SurfaceMisses, Ocean->SurfaceMaxErr, Ocean->SurfaceMaxEta);
	if (Ocean->SurfaceChecks < 3) { Problems.Add(FString::Printf(TEXT("too few surface checks: %d"), Ocean->SurfaceChecks)); }
	if (Ocean->SurfaceMisses > 0) { Problems.Add(FString::Printf(TEXT("surface not drawn at %lld points"), Ocean->SurfaceMisses)); }
	if (Ocean->SurfaceMaxErr > 5.0e-3) { Problems.Add(FString::Printf(TEXT("drawn surface differs from the physics wave: %.2e m"), Ocean->SurfaceMaxErr)); }
	if (Ocean->SurfaceMaxEta < 0.1) { Problems.Add(TEXT("surface check saw no waves")); }
}

bool AShipFleet::WriteSamples(double PhysicsTime)
{
	// アクタの変換から読み戻した値（ShipFrame の逆変換）。物理の trace と同じ列。
	// 照合（tools/compare_plant_trace.py）は trace を同じ式で補間するので、補間の両端が物理の隣り合うステップのときだけ書く
	// （物理が詰まって何ステップか一度に届くと、PDU は最新しか読めず、間を飛ばして補間する。それは正しい表示だが trace とは比べられない）
	for (const FShipSync& S : Sync)
	{
		double Alpha = 0.0;
		const int32 i = Bracket(S.Poses, PhysicsTime, Alpha);
		if (S.PosFrames > 0 && (i == INDEX_NONE || i + 1 >= S.Poses.Num() || S.Poses[i + 1].T - S.Poses[i].T > 0.0201))
		{
			++SamplesSkipped;
			return false;
		}
	}
	for (const FShipSync& S : Sync)
	{
		if (S.PosFrames == 0)
		{
			continue;
		}
		FVector L, A;
		S.Actor->ReadBackPose(L, A);
		SampleLines.Add(FString::Printf(TEXT("%.6f,%s,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f"), PhysicsTime, *S.Actor->Robot, L.X, L.Y, L.Z, A.X, A.Y, A.Z));
	}
	return true;
}

void AShipFleet::LogStatus(double PhysicsTime)
{
	const double Wall = FPlatformTime::Seconds();
	if (LogWallStart < 0.0)
	{
		LogWallStart = Wall;
	}
	++LogFrames;
	// fps の内訳（エンジンの stat unit と同じ値）：ゲームスレッド・描画スレッド・GPU の時間 [ms] と、窓が前面だった割合
	LogGameMs += FPlatformTime::ToMilliseconds(GGameThreadTime);
	LogRenderMs += FPlatformTime::ToMilliseconds(GRenderThreadTime);
	LogGpuMs += FPlatformTime::ToMilliseconds(RHIGetGPUFrameCycles());
	LogFocus += FApp::HasFocus() ? 1 : 0;
	if (LastFrameWall > 0.0)
	{
		const double Ms = (Wall - LastFrameWall) * 1000.0;
		LogMaxFrameMs = FMath::Max(LogMaxFrameMs, Ms);
		LogSlowFrames += Ms > 15.0 ? 1 : 0;
	}
	LastFrameWall = Wall;
	if (Wall - LogWallStart < 5.0)
	{
		return;
	}
	TArray<FString> Parts;
	for (const FShipSync& S : Sync)
	{
		Parts.Add(FString::Printf(TEXT("%s pos=%lld (%.1f, %.1f, %.2f) yaw %.1f deg%s"), *S.Actor->Robot, S.PosFrames,
			S.LastLinear.X, S.LastLinear.Y, S.LastLinear.Z, FMath::RadiansToDegrees(S.LastAngular.Z),
			S.JointPdu.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(" joints=%lld"), S.JointFrames)));
	}
	const double N = FMath::Max<double>(LogFrames, 1.0);
	UE_LOG(LogShipFleet, Log, TEXT("physics t=%.2f s  fps %.0f (game %.1f render %.1f gpu %.1f ms, max frame %.0f ms, >15 ms %lld, max tick rate %.0f, focus %.0f%%)  %s"),
		PhysicsTime, LogFrames / (Wall - LogWallStart), LogGameMs / N, LogRenderMs / N, LogGpuMs / N, LogMaxFrameMs, LogSlowFrames,
		GEngine ? GEngine->GetMaxTickRate(0.0f, false) : -1.0f, 100.0 * LogFocus / N, *FString::Join(Parts, TEXT(" | ")));
	LogWallStart = Wall;
	LogFrames = 0;
	LogGameMs = LogRenderMs = LogGpuMs = 0.0;
	LogFocus = 0;
	LogMaxFrameMs = 0.0;
	LogSlowFrames = 0;
}

void AShipFleet::FinishCheck(double PhysicsTime)
{
	bDone = true;
	TArray<FString> Problems;
	double PosErr = 0.0, AngErr = 0.0, JointErr = 0.0;
	for (const FShipSync& S : Sync)
	{
		if (S.PosFrames == 0) { Problems.Add(FString::Printf(TEXT("%s: no %s"), *S.Actor->Robot, *S.PosPdu)); }
		if (!S.JointPdu.IsEmpty() && S.JointFrames == 0) { Problems.Add(FString::Printf(TEXT("%s: no %s"), *S.Actor->Robot, *S.JointPdu)); }
		PosErr = FMath::Max(PosErr, S.MaxPosErr);
		AngErr = FMath::Max(AngErr, S.MaxAngErr);
		JointErr = FMath::Max(JointErr, S.MaxJointErr);
	}
	// 位置は倍精度（UE5 の大きな世界座標）、向き・関節は単精度の四元数の丸め程度
	if (PosErr > 1.0e-3) { Problems.Add(FString::Printf(TEXT("position/pivot error %.6f m > 1e-3"), PosErr)); }
	if (AngErr > 1.0e-4) { Problems.Add(FString::Printf(TEXT("axis error %.3e > 1e-4"), AngErr)); }
	if (JointErr > 1.0e-4) { Problems.Add(FString::Printf(TEXT("joint error %.3e rad > 1e-4"), JointErr)); }
	CheckSea(Problems);
	// 表示時刻のなめらかさ：止まったフレーム（新しい物理が届かない間）の割合
	const double HoldRatio = DisplayFrames > 0 ? static_cast<double>(DisplayHolds) / DisplayFrames : 1.0;
	UE_LOG(LogShipFleet, Log, TEXT("SHIP_VIEW display clock: %lld frames, held %lld (%.2f %%), resyncs %lld, delay %.3f s, max lag behind physics %.3f s"),
		DisplayFrames, DisplayHolds, 100.0 * HoldRatio, ClockSnaps, DisplayDelay, DisplayLagMax);
	if (HoldRatio > 0.05) { Problems.Add(FString::Printf(TEXT("display clock held in %.1f %% of frames (not smooth)"), 100.0 * HoldRatio)); }
	if (!SamplesPath.IsEmpty())
	{
		const FString Csv = TEXT("t,robot,x,y,z,roll,ang_y,yaw\n") + FString::Join(SampleLines, TEXT("\n")) + TEXT("\n");
		if (!FFileHelper::SaveStringToFile(Csv, *SamplesPath))
		{
			Problems.Add(FString::Printf(TEXT("cannot write %s"), *SamplesPath));
		}
	}
	if (!FrameLogPath.IsEmpty())
	{
		FFileHelper::SaveStringToFile(TEXT("wall,dt,physics,new,display\n") + FString::Join(FrameLines, TEXT("\n")) + TEXT("\n"), *FrameLogPath);
	}
	TArray<FString> Counts;
	for (const FShipSync& S : Sync)
	{
		Counts.Add(FString::Printf(TEXT("%s=%lld"), *S.Actor->Robot, S.PosFrames));
	}
	const FString Summary = FString::Printf(TEXT("display %.2f..%.2f s (physics stalls %lld) frames %s max err pos %.2e m axis %.2e joint %.2e rad samples %d (skipped frames %lld)"),
		FirstDisplayTime, PhysicsTime, ClockStalls, *FString::Join(Counts, TEXT(" ")), PosErr, AngErr, JointErr, SampleLines.Num(), SamplesSkipped);
	if (Problems.Num() == 0)
	{
		UE_LOG(LogShipFleet, Display, TEXT("HAKONIWA_UNREAL_SHIP_SYNC_OK %s"), *Summary);
	}
	else
	{
		UE_LOG(LogShipFleet, Error, TEXT("HAKONIWA_UNREAL_SHIP_SYNC_FAILED %s : %s"), *Summary, *FString::Join(Problems, TEXT("; ")));
	}
	ExitCode = Problems.Num() == 0 ? 0 : 1;
	if (ShotPath.IsEmpty())
	{
		FPlatformMisc::RequestExitWithStatus(false, ExitCode);
		return;
	}
	// 撮影は次の描画で行われるので、数フレーム待ってから終わる
	FScreenshotRequest::RequestScreenshot(FPaths::ConvertRelativePathToFull(ShotPath), true, false);   // 画面の警告（赤い文字）も写す
	UE_LOG(LogShipFleet, Log, TEXT("shot -> %s"), *ShotPath);
	ExitCountdown = 30;
}

void AShipFleet::UpdateChase(float DeltaSeconds)
{
	// 自船（最初の船）の後ろ 160 m・上 45 m から、船の少し上を見る。向きは方位だけ（揺れは追わない）でなめらかに
	if (Ships.Num() == 0 || !Ships[0])
	{
		return;
	}
	const AShipActor* Own = Ships[0];
	const FVector Fwd = FVector(Own->GetActorForwardVector().X, Own->GetActorForwardVector().Y, 0.0).GetSafeNormal();
	const FVector Right = FVector::CrossProduct(FVector::UpVector, Fwd);
	const FVector Target = Own->GetActorLocation() + FVector(0, 0, 800.0);
	const FVector Want = Own->GetActorLocation() - Fwd * 16000.0 + Right * 6000.0 + FVector(0, 0, 4500.0);
	const float A = 1.0f - FMath::Exp(-DeltaSeconds * 2.0f);
	const FVector Now = ChaseCam->GetComponentLocation();
	const FVector Pos = (Now - Want).Size() > 50000.0 ? Want : FMath::Lerp(Now, Want, A);
	ChaseCam->SetWorldLocationAndRotation(Pos, (Target - Pos).Rotation());
	if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0); PC && PC->GetViewTarget() != this)
	{
		PC->SetViewTarget(this);
	}
}

void AShipFleet::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (bChase)
	{
		UpdateChase(DeltaSeconds);
	}
	if (ExitCountdown >= 0 && --ExitCountdown < 0)
	{
		FPlatformMisc::RequestExitWithStatus(false, ExitCode);
	}
	if (!bSync || bDone)
	{
		return;
	}
	UPduManager* Pdu = bDeclared ? FindPduManager() : nullptr;
	if (!Pdu || !Pdu->IsServiceEnabled())
	{
		return;
	}
	if (bHakoStart && !bStartRequested)
	{
		// Godot ビューアの request_start と同じ：登録と宣言が済んだら開始を要求する（conductor と PLANT は先に動いている前提）
		bStartRequested = true;
		if (AActor* Client = FindClient())
		{
			UE_LOG(LogShipFleet, Log, TEXT("requesting simulation start"));
			IHakoniwaClientInterface::Execute_Start(Client);
		}
	}
	// ★ PduManager::ReadPduRawData は受信した PDU を 1 回だけ返す（CommunicationBuffer::GetBuffer が取り出して消す）。
	//   新しい値が無いフレームは読めない。受信した値は物理時刻つきの履歴に積み、描くのは表示時刻（UpdateDisplayClock）の補間値
	//   ship_plant は 1 ステップで全船の pos を書いてから、時刻（stamp）つきの joint_states を書く。pos には時刻が無いので、
	//   先に全船の pos を読み、そのあと時刻を読む。新しい時刻が届くまでに読めた pos はその時刻のもの（届かなければ次の時刻まで待たせる）
	double Stamp = -1.0;
	for (FShipSync& S : Sync)
	{
		ReadPose(Pdu, S);
	}
	for (FShipSync& S : Sync)
	{
		double St = -1.0;
		if (ReadJoints(Pdu, S, St) && S.Actor->Robot == ClockRobot && S.JointPdu == ClockPdu)
		{
			Stamp = St;
		}
	}
	ReadDisturbance(Pdu);
	if (Stamp < 0.0)
	{
		// 新しい joint_states が無いフレーム（描画が物理の 50 Hz より速いと大半）
	}
	else if (Stamp <= LastPhysicsTime)
	{
		++ClockStalls;   // 物理時刻が進んでいない joint_states（PLANT が同じ時刻で書き直した）
	}
	else
	{
		LastPhysicsTime = Stamp;
		if (FirstPhysicsTime < 0.0)
		{
			FirstPhysicsTime = Stamp;
			UE_LOG(LogShipFleet, Log, TEXT("first physics stamp %.3f s"), Stamp);
		}
	}
	if (LastPhysicsTime < 0.0)
	{
		return;
	}
	if (Stamp >= 0.0 && Stamp == LastPhysicsTime)
	{
		for (FShipSync& S : Sync)
		{
			PushSamples(S, LastPhysicsTime);
		}
	}
	UpdateDisplayClock(DeltaSeconds);
	if (FirstDisplayTime < 0.0)
	{
		FirstDisplayTime = DisplayTime;
	}
	for (FShipSync& S : Sync)
	{
		ApplyAt(S, DisplayTime);
	}
	// 標本は始まって 1 s 後から（開始直後は物理が遅れを取り戻すため時刻が数ステップずつ飛び、最初の pos の時刻が決められない）
	if (!SamplesPath.IsEmpty() && DisplayTime >= FirstDisplayTime + 1.0 && DisplayTime - LastSampleTime >= SampleInterval - 1e-9)
	{
		if (WriteSamples(DisplayTime))
		{
			LastSampleTime = DisplayTime;
		}
	}
	if (Ocean)
	{
		// 海面は船と同じ表示時刻で描く（D19：受信した姿勢と同じ物理時刻の海面。補間した姿勢には同じ時刻の海面）
		Ocean->SetPhysicsTime(DisplayTime);
		++ClockFrames;
		ClockFromPhysics += Ocean->GetTime() == DisplayTime ? 1 : 0;
		if (CheckSec > 0.0 && DisplayTime > SettleSec)
		{
			SampleHeave();
			if (DisplayTime - LastSurfaceRequest >= 10.0)
			{
				LastSurfaceRequest = DisplayTime;
				Ocean->RequestSurfaceCheck();
			}
		}
	}
	if (!FrameLogPath.IsEmpty())
	{
		FrameLines.Add(FString::Printf(TEXT("%.6f,%.6f,%.6f,%d,%.6f"), FPlatformTime::Seconds(), DeltaSeconds, LastPhysicsTime, Stamp >= 0.0 ? 1 : 0, DisplayTime));
	}
	LogStatus(LastPhysicsTime);
	if (CheckSec > 0.0 && DisplayTime - FirstDisplayTime >= CheckSec)
	{
		FinishCheck(DisplayTime);
	}
}
