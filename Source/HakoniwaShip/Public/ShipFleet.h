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
//   -ShipChaseToward=light    追うカメラを太陽（夜は月）の方へ向ける（まぶしさ・照り返しの確認、Step 5）
//   -ShipNavLightCheck        夜（-ShipNight）に、各船の航海灯を周り 36 方位から撮り、射光範囲（COLREG 第 21 条）の内だけ光るかを確かめる
//                             （HAKONIWA_UNREAL_SHIP_NAVLIGHT_OK / _FAILED。始まって 60 フレーム後に 1 回）
//   -ShipCameraDump=<dir>     船載カメラ（SHP-25）の画像を 2 s ごとに PNG で保存（箱庭につながないときもこれで撮る）
//   -ShipView=free|chase|bridge|foc  始めの画面（SHP-25 Step 2。実行中はキー 1 追う・2 船橋・3 FOC・0 自由）
//   -ShipShotViews=chase,bridge,foc  -ShipShot の代わりに、終わりに画面ごとに撮る（<ShipShot の名前>_<画面>.png）
//   -ShipHudLog=<csv>         ブリッジ視界・FOC に出す航海情報を 1 s ごとに書く（tools/compare_nav_hud.py が物理の trace と照合する）
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
#include "ShipNav.h"
#include "ShipWave.h"
#include "ShipFleet.generated.h"

class AShipActor;
class AShipLighting;
class UShipCamera;
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
	bool bWrite = false;   // ビューアが書く PDU（船載カメラ、SHP-25）
};

/** 画面（SHP-25 Step 2）：自由（観客）・自船を追う・船橋からの視界・FOC（陸上監視センター） */
UENUM()
enum class EShipView : uint8 { Free, Chase, Bridge, Foc };

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

	// ---- ブリッジ視界・FOC（SHP-25 Step 2）が読む値。毎フレーム表示時刻で更新する ----
	struct FShipView
	{
		FString Robot;
		FString Name;        // AIS の船名（無ければ空）
		bool bOwn = false;
		bool bAis = false;
		double LengthM = 0.0;
		ShipNav::FState State;
		TArray<FVector2D> Trail;   // 航跡（東, 北）[m]、1 s ごと
	};
	TArray<FShipView> ViewShips;          // Ships と同じ順。0 番が自船
	TArray<ShipNav::FTarget> Targets;     // 自船から見た他船（ViewShips[1..] の順）
	double RudderDeg = 0.0;               // 舵角（右舵が正。物理の舵角は左舵正）
	double PropRpm = 0.0;                 // プロペラ回転数
	bool bHasJoints = false;
	double ViewTime = -1.0;               // 表示の時刻（物理時刻の軸）
	// 自動化レベル（autonomy_status ch 50、std_msgs/String の JSON、D29）。SUT（ship_sut）が 5 Hz で書く
	int64 AutonomyFrames = 0;             // 受け取った数
	double AutonomyLastT = -1.0;          // 最後に受け取った物理時刻
	int32 AutonomyLevel = 0;              // al：4 自律・3 監視・2 縮退・1 遠隔/手動（0 は不明）
	double AutonomyOddMargin = -1.0;      // odd：ODD 余裕度 0〜1
	bool bAutonomyMrs = false;            // mrs：最小リスク状態
	FString AutonomyReason;               // reason
	FString AutonomyNav;                  // nav：NORMAL・AVOID_STBD・STAND_ON_ACT・SLOW・RETURN
	double AutonomyCourseDeg = -1.0;      // crs：指令針路 [deg]
	double AutonomySpeedKt = -1.0;        // spd：指令速力 [kt]
	int32 AutonomyThreat = 0;             // tgt：主な避航対象の追尾番号（0 は無し）
	double VectorMinutes = 6.0, OztSafeDomainM = 926.0, OztHorizonS = 900.0, DisplayRangeM = 3000.0;
	AShipOcean* GetOcean() const { return Ocean; }
	const TArray<TObjectPtr<UShipCamera>>& GetCameras() const { return Cameras; }
	bool IsSynced() const { return bSync; }
	EShipView GetView() const { return View; }
	void SetView(EShipView V);

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
		TArray<ShipNav::FSample> NavHist;   // 航海情報の推定に使う受信値（物理時刻つき、窓 + 1 s 分）
		double LastTrailT = -1.0e9;
		TArray<double> LastJointVel;        // 最後に受け取った関節の速度 [rad/s]
		TArray<double> AppliedJoints;       // 表示時刻で置いた関節の角度 [rad]
		TArray<FString> JointNames;
		bool bNewPose = false;
		FVector NewLinear = FVector::ZeroVector;
		FVector NewAngular = FVector::ZeroVector;
		bool bNewJoints = false;
		TArray<double> NewJoints;
		TArray<double> NewJointVel;
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
	void UpdateViewData(double T);
	void WriteHudLog(double T);
	void TickShotViews();
	void CheckNavLights();

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
	int64 ClockSnaps = 0;         // 2 s 以上ずれて合わせ直した回数
	double DisplayLagMax = 0.0;
	int64 Hitches = 0;            // 始まって 5 s 後からの 25 ms を超えたフレーム（目に見えるカクつき）
	int64 HitchFrames = 0;
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
	EShipView View = EShipView::Free;
	bool bInitialView = false;
	int32 WarmFrames = 0;   // 開始の要求の前に、続けて 25 ms 未満だったフレームの数
	double VelocityWindow = 5.0;
	FString HudLogPath;
	TArray<FString> HudLines;
	double NextHudLog = 0.0;
	TArray<EShipView> ShotViews;   // 終わりに撮る画面（-ShipShotViews）
	int32 ShotStep = -1;
	int32 ShotWait = 0;
	FString AutonomyPdu;           // autonomy_status（読むだけ）
	bool bChaseTowardLight = false;
	bool bNavLightCheck = false;
	int32 NavLightCheckFrames = 0;
	int32 ExitCountdown = -1;
	int32 ExitCode = 0;

	UPROPERTY() TObjectPtr<UCameraComponent> ChaseCam;
	UPROPERTY() TObjectPtr<UCameraComponent> BridgeCam;   // 自船の船橋カメラの位置（人の目の画角）
	UPROPERTY() TObjectPtr<AShipOcean> Ocean;
	UPROPERTY() TObjectPtr<AShipLighting> Lighting;
	UPROPERTY() TArray<TObjectPtr<UShipCamera>> Cameras;   // 船載カメラ（SHP-25。fleet_ue.json の cameras）
	bool bCamerasOffline = false;                         // つながないときも撮る（-ShipCameraDump）
};
