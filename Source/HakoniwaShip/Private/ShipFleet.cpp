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
#include "UnrealClient.h"

#include "pdu_convertor.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
// この版の hakoniwa-pdu-registry は文字列の可変長配列の変換を ros 名でだけ持つ（hakoniwa-unreal-rover の RoverAvatar と同じ読み替え）
#define hako_convert_pdu2cpp_array_string_varray hako_convert_pdu2ros_array_string_varray
#define hako_convert_cpp2pdu_array_string_varray hako_convert_ros2pdu_array_string_varray
#include "sensor_msgs/pdu_cpptype_conv_JointState.hpp"
#undef hako_convert_pdu2cpp_array_string_varray
#undef hako_convert_cpp2pdu_array_string_varray

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
	const FVector Lin(T.linear.x, T.linear.y, T.linear.z);
	const FVector Ang(T.angular.x, T.angular.y, T.angular.z);
	S.Actor->ApplyPose(Lin, Ang);
	S.LastLinear = Lin;
	S.LastAngular = Ang;
	++S.PosFrames;

	// 検証 1：Unreal のアクタの位置と軸（エンジンが四元数から求めた前・右・上の向き）を、PDU から右手系で直接求めた値と比べる。
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
	TArray<FString> Names;
	TArray<double> Pos;
	for (size_t i = 0; i < J.name.size(); ++i)
	{
		Names.Add(UTF8_TO_TCHAR(J.name[i].c_str()));
		Pos.Add(i < J.position.size() ? J.position[i] : 0.0);
	}
	S.Actor->ApplyJoints(Names, Pos);
	++S.JointFrames;
	OutStamp = J.header.stamp.sec + J.header.stamp.nanosec * 1.0e-9;
	// 検証 3：関節の部品の角度を読み戻す（軸まわりの回転角。2π の折り返しは除く）
	for (int32 i = 0; i < Names.Num(); ++i)
	{
		double Q = 0.0;
		if (S.Actor->ReadBackJoint(Names[i], Q))
		{
			S.MaxJointErr = FMath::Max(S.MaxJointErr, FMath::Abs(WrapPi(Q - Pos[i])));
		}
	}
	return true;
}

void AShipFleet::WriteSamples(double PhysicsTime)
{
	// アクタの変換から読み戻した値（ShipFrame の逆変換）。物理の trace と同じ列
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
	if (!SamplesPath.IsEmpty())
	{
		const FString Csv = TEXT("t,robot,x,y,z,roll,ang_y,yaw\n") + FString::Join(SampleLines, TEXT("\n")) + TEXT("\n");
		if (!FFileHelper::SaveStringToFile(Csv, *SamplesPath))
		{
			Problems.Add(FString::Printf(TEXT("cannot write %s"), *SamplesPath));
		}
	}
	TArray<FString> Counts;
	for (const FShipSync& S : Sync)
	{
		Counts.Add(FString::Printf(TEXT("%s=%lld"), *S.Actor->Robot, S.PosFrames));
	}
	const FString Summary = FString::Printf(TEXT("physics %.2f..%.2f s (stalls %lld) frames %s max err pos %.2e m axis %.2e joint %.2e rad samples %d"),
		FirstPhysicsTime, PhysicsTime, ClockStalls, *FString::Join(Counts, TEXT(" ")), PosErr, AngErr, JointErr, SampleLines.Num());
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
	FScreenshotRequest::RequestScreenshot(FPaths::ConvertRelativePathToFull(ShotPath), false, false);
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
	//   新しい値が無いフレームは読めないので、アクタは前の姿勢のまま。fps はここで読めたかどうかに関係なく数える
	double Stamp = -1.0;
	for (FShipSync& S : Sync)
	{
		ReadPose(Pdu, S);
		double St = -1.0;
		if (ReadJoints(Pdu, S, St) && S.Actor->Robot == ClockRobot && S.JointPdu == ClockPdu)
		{
			Stamp = St;
		}
	}
	if (Stamp < 0.0)
	{
		// 新しい joint_states が無いフレーム（描画が物理の 50 Hz より速いと大半）
	}
	else if (Stamp <= LastPhysicsTime)
	{
		++ClockStalls;   // 物理時刻が進んでいない joint_states（PLANT が同じ時刻で書き直した）。標本は書かない
	}
	else
	{
		LastPhysicsTime = Stamp;
		if (FirstPhysicsTime < 0.0)
		{
			FirstPhysicsTime = Stamp;
			UE_LOG(LogShipFleet, Log, TEXT("first physics stamp %.3f s"), Stamp);
		}
		if (!SamplesPath.IsEmpty() && Stamp - LastSampleTime >= SampleInterval - 1e-9)
		{
			LastSampleTime = Stamp;
			WriteSamples(Stamp);
		}
	}
	LogStatus(LastPhysicsTime);
	if (CheckSec > 0.0 && FirstPhysicsTime >= 0.0 && LastPhysicsTime - FirstPhysicsTime >= CheckSec)
	{
		FinishCheck(LastPhysicsTime);
	}
}
