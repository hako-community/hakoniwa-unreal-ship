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
//   -ShipShot=<png>           -ShipSyncCheck の終わりに画面を撮ってから終わる（目視確認用）
//
// ★ HAKO_CONFIG_PATH（共有メモリ Z:/mmap_ship）と shakoc.dll の場所は scripts\windows\ship_env.ps1（ship_scene.ps1・run_plant_test.ps1 が読む）が設定する。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HakoniwaObjectInterface.h"
#include "ShipFleet.generated.h"

class AShipActor;
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
	};

	bool LoadFleet();
	FString WritePduConfig() const;
	UPduManager* FindPduManager() const;
	AActor* FindClient() const;
	bool ReadPose(UPduManager* Pdu, FShipSync& S);
	bool ReadJoints(UPduManager* Pdu, FShipSync& S, double& OutStamp);
	void WriteSamples(double PhysicsTime);
	void LogStatus(double PhysicsTime);
	void FinishCheck(double PhysicsTime);
	void UpdateChase(float DeltaSeconds);

	FString AssetName = TEXT("UnrealShipViewer");
	FString ClockRobot;
	FString ClockPdu;
	TArray<FShipPduRead> PduTable;   // 読む PDU（fleet_ue.json の pdus）
	TArray<FShipSync> Sync;
	TArray<uint8> Buf;

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
	bool bChase = false;
	int32 ExitCountdown = -1;
	int32 ExitCode = 0;

	UPROPERTY() TObjectPtr<UCameraComponent> ChaseCam;
};
