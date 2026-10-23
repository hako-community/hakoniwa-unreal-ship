// ShipFleet.h
// 船団（SHP-24 Step 2）。Config/Fleet/fleet_ue.json の船を AShipActor として並べ、箱庭の PDU（D14・D18）で動かす。
// hakoniwa-unreal-rover の ARoverAvatar と同じく、-ShipSync のときだけ AHakoniwaShmClient を置いてアセットとして登録し、
// OnHakoInitialize で読む PDU を宣言し、Tick で PduManager から読む。
//
// コマンドライン（-game のとき）:
//   -ShipSync                 箱庭につなぐ（無ければ fleet_ue.json の preview_pose に置いて止めておく）
//   -ShipHakoStart            登録のあとシミュレーションの開始を要求する（Godot ビューアの --plant-test と同じ。hako_simevent_start）
//   -ShipSyncCheck=<秒>       物理時刻でその秒数だけ受信したら、受信数と読み戻しの誤差を
//                             HAKONIWA_UNREAL_SHIP_SYNC_OK / HAKONIWA_UNREAL_SHIP_SYNC_FAILED としてログに書いて終わる
//   -ShipSamples=<csv>        受信した姿勢をアクタから読み戻した値（ShipFrame の逆変換）で書く。
//                             列は ship_plant の --trace と同じ（t,robot,x,y,z,roll,ang_y,yaw）。t は物理時刻（OwnShip の joint_states の stamp、D19）
//                             tools/compare_plant_trace.py が物理の trace と照合する
//   -ShipSampleInterval=<秒>  書く間隔（物理時刻、既定 0.5）
//   -ShipChase                自船を後ろから追うカメラで見る（-ShipShot のときも）
//   -ShipShot=<png>           -ShipSyncCheck の終わりに画面を撮ってから終わる（目視確認用。画面の警告の文字も写る）
//   -ShipFrameLog=<csv>       フレームごとの壁時計・物理時刻・表示時刻（カクつきの調査用。-ShipSyncCheck の終わりに書く）
//
// 表示時刻（Step 3）：物理は 50 Hz、描画は 130〜200 FPS。受信した値をそのまま描くと 2〜4 フレームごとに飛んでカクつくので、
// 受信した値（物理時刻つき）を履歴に積み、なめらかに進む表示時刻（最新の物理時刻の 0.04 s 後ろ）で補間して描く。
// 船・関節・海面はすべて同じ表示時刻（D19 の考え方：船の姿勢と海面は同じ物理時刻）。
//
// 海面（Step 3）：地図の AShipOcean に、Disturbance PDU の波浪パラメータ（無ければ fleet_ue.json の wave.preview。箱庭につないだときは
// 届くまで静水）と物理時刻（D19）を渡す。-ShipSyncCheck では Godot の結合テストと同じく、海面と船体の上下動の相関・水線、
// 海面の時刻が物理時刻か、波浪パラメータ、描いた海面の高さ（AShipOcean の真上からの検証）も確かめる。
//
// ★ HAKO_CONFIG_PATH（共有メモリ Z:/mmap_ship）と shakoc.dll の場所は scripts\windows\ship_env.ps1（ship_scene.ps1・run_plant_test.ps1 が読む）が設定する。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HakoniwaObjectInterface.h"
#include "ShipWave.h"
#include "ShipFleet.generated.h"

class AShipActor;
class AShipOcean;
class UCameraComponent;
class UPduManager;

USTRUCT()
struct FShipPduRead
{
	GENERATED_BODY()

	FString Robot;
	FString Pdu;
	FString Type;
	int32 Channel = 0;
	int32 Size = 0;
};

UCLASS()
class HAKONIWASHIP_API AShipFleet : public AActor, public IHakoniwaObjectInterface
{
	GENERATED_BODY()

public:
	AShipFleet();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	virtual void OnHakoInitialize_Implementation() override;
	virtual void OnHakoReset_Implementation() override {}

	/** fleet_ue.json（プロジェクトからの相対） */
	UPROPERTY(EditAnywhere, Category = "Ship") FString FleetConfig = TEXT("Config/Fleet/fleet_ue.json");

	UPROPERTY(VisibleAnywhere, Category = "Ship") TArray<TObjectPtr<AShipActor>> Ships;

private:
	/** 船体の上下位置 z と海面 η（船体の水線面の平均）の相関の累積（Godot の plant_coupling_check.gd と同じ） */
	struct FHeaveAcc
	{
		double Sx = 0.0, Sy = 0.0, Sxx = 0.0, Syy = 0.0, Sxy = 0.0, Sd2 = 0.0;
		int64 N = 0;
		double Corr() const;
	};

	/** 受信した値（物理時刻 T）。姿勢は Lin / Ang、関節は Joints */
	struct FShipSample
	{
		double T = 0.0;
		FVector Lin = FVector::ZeroVector;
		FVector Ang = FVector::ZeroVector;
		TArray<double> Joints;
	};

	struct FShipSync
	{
		AShipActor* Actor = nullptr;
		FString PosPdu;
		FString JointPdu;
		int64 PosFrames = 0;
		int64 JointFrames = 0;
		double MaxPosErr = 0.0;   // 読み戻しの位置の誤差 [m]
		double MaxAngErr = 0.0;   // 読み戻しの姿勢の誤差 [rad]
		double MaxJointErr = 0.0; // 読み戻しの関節角の誤差 [rad]（2π の折り返しは除く）
		FVector LastLinear = FVector::ZeroVector;
		FVector LastAngular = FVector::ZeroVector;
		double LppM = 0.0;   // 水線面の範囲（plant_check。0 なら上下動を検証しない）
		double BeamM = 0.0;
		TArray<FHeaveAcc> Heave;   // HeaveLags ごと
		// 受信の履歴（表示時刻で補間する）と、このフレームで読めた値
		TArray<FShipSample> Poses;
		TArray<FShipSample> JointHist;
		TArray<FString> JointNames;
		bool bNewPose = false;
		FVector NewLinear = FVector::ZeroVector;
		FVector NewAngular = FVector::ZeroVector;
		bool bNewJoints = false;
		TArray<double> NewJoints;
	};

	bool LoadFleet();
	FString WritePduConfig() const;
	UPduManager* FindPduManager() const;
	AActor* FindClient() const;
	bool ReadPose(UPduManager* Pdu, FShipSync& S);
	bool ReadJoints(UPduManager* Pdu, FShipSync& S, double& OutStamp);
	void ReadDisturbance(UPduManager* Pdu);
	void PushSamples(FShipSync& S, double T);
	static int32 Bracket(const TArray<FShipSample>& H, double T, double& OutAlpha);
	void ApplyAt(FShipSync& S, double T);
	void UpdateDisplayClock(double Dt);
	void SampleHeave();
	void CheckSea(TArray<FString>& Problems);
	bool WriteSamples(double PhysicsTime);
	void LogStatus(double PhysicsTime);
	void FinishCheck(double PhysicsTime);
	void UpdateChase(float DeltaSeconds);

	FString AssetName = TEXT("UnrealShipViewer");
	FString ClockRobot;
	FString ClockPdu;
	TArray<FShipPduRead> PduTable;   // 読む PDU（fleet_ue.json の pdus）
	TArray<FShipSync> Sync;
	TArray<uint8> Buf;

	// 海面（Step 3）
	FString DistRobot;
	FString DistPdu;
	int32 WaveComponents = ShipWave::DefaultComponents;
	double WaveActiveHs = ShipWave::DefaultActiveHs;
	ShipWave::FParams PreviewWave;
	bool bHasPreviewWave = false;
	int64 DistFrames = 0;
	// 結合テストの期待値（fleet_ue.json の plant_check。Godot の expected.json と同じ値）
	double SettleSec = 20.0;
	double HeaveCorrMin = 0.8;
	double WaterlineRmsMax = 0.4;
	ShipWave::FParams ExpectedWave;
	bool bHasExpectedWave = false;
	TArray<double> HeaveLags;   // [0, 0.5, 1.0, 1.5, Tp/2, −Tp/2] s
	int64 ClockFrames = 0;      // 物理時刻を受け取ってからのフレーム
	int64 ClockFromPhysics = 0; // そのうち海面が物理時刻で描かれたフレーム
	double LastSurfaceRequest = -1.0e9;

	// 表示時刻（なめらかに進める。船・関節・海面はこの時刻で描く）
	double DisplayDelay = 0.04;   // 最新の物理時刻からの遅れ [s]（物理 2 ステップ。受信の間隔の揺れを吸収する）
	double DisplayTime = -1.0;
	double FirstDisplayTime = -1.0;
	int64 DisplayFrames = 0;
	int64 DisplayHolds = 0;       // 表示時刻が進まなかったフレーム
	int64 ClockSnaps = 0;         // 0.5 s 以上ずれて合わせ直した回数
	double DisplayLagMax = 0.0;
	int64 SamplesSkipped = 0;     // 補間の両端が隣り合うステップでないため標本を書かなかったフレーム

	bool bSync = false;
	bool bHakoStart = false;
	bool bStartRequested = false;
	bool bDeclared = false;
	bool bDone = false;
	double CheckSec = -1.0;
	double FirstPhysicsTime = -1.0;
	double LastPhysicsTime = -1.0;
	double LastSampleTime = -1.0e9;
	double SampleInterval = 0.5;
	double LogWallStart = -1.0;
	int64 LogFrames = 0;
	double LogGameMs = 0.0;
	double LogRenderMs = 0.0;
	double LogGpuMs = 0.0;
	int64 LogFocus = 0;
	double LogMaxFrameMs = 0.0;
	int64 LogSlowFrames = 0;
	double LastFrameWall = -1.0;
	int64 ClockStalls = 0;
	FString SamplesPath;
	TArray<FString> SampleLines;
	FString ShotPath;
	FString FrameLogPath;          // -ShipFrameLog=<csv>：フレームごとの壁時計・物理時刻（カクつきの調査用）
	TArray<FString> FrameLines;
	bool bChase = false;
	int32 ExitCountdown = -1;
	int32 ExitCode = 0;

	UPROPERTY() TObjectPtr<UCameraComponent> ChaseCam;
	UPROPERTY() TObjectPtr<AShipOcean> Ocean;
};
