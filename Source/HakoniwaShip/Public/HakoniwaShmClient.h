#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PduManager.h"
#include "ShmCommunicationService.h"
#include "HakoniwaClientInterface.h"
#include "HakoniwaShmClient.generated.h"

class FHakoniwaTimeSyncWorker;
class FRunnableThread;
class IInputProcessor;

UCLASS(Blueprintable, BlueprintType)
class HAKONIWASHIP_API AHakoniwaShmClient : public AActor, public IHakoniwaClientInterface
{
    GENERATED_BODY()

public:
    AHakoniwaShmClient();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hakoniwa")
    FString ConfigPath = "Config/webavatar.json";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hakoniwa")
    FString AssetName = "UnrealAsset";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hakoniwa")
    bool bAutoInitialize = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hakoniwa|Time Sync")
    bool bEnableRealTimePacing = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hakoniwa|Time Sync", meta = (ClampMin = "0.01"))
    float TargetRealTimeFactor = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hakoniwa|Time Sync", meta = (ClampMin = "1", ClampMax = "20"))
    int32 TimeSyncIntervalMsec = 5;

    /// ★ 2026-10-04: オンライン（共有メモリ）では**ゲームパッドを Unreal に読ませない**。
    ///   機体はプラント（swarm_app / drone-core）が動かし、ゲームパッドは外の送信機（rc_pdu_pub.py）が読む。
    ///   Unreal も読むと、○×△□ が画面の START / STOP / RESET ボタンを押してしまい（UI のゲームパッド操作）、
    ///   スティックは視点を回す（「ぐるぐる画面が回るだけでドローンが飛ばない」）。キーボードとマウスはそのまま。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Hakoniwa")
    bool bIgnoreGamepad = true;

    virtual void Start_Implementation() override;
    virtual void Stop_Implementation() override;
    virtual void Reset_Implementation() override;

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime) override;

    UFUNCTION(BlueprintCallable, Category = "Hakoniwa")
    bool InitializeClient();

    virtual UPduManager* GetPduManager_Implementation() const override { return pduManager; }
    virtual int32 GetSimulationState_Implementation() const override;


private:
    UPROPERTY()
    UShmCommunicationService* service = nullptr;
    UPROPERTY()
    UPduManager* pduManager = nullptr;

    FHakoniwaTimeSyncWorker* TimeSyncWorker = nullptr;
    TSharedPtr<IInputProcessor> GamepadBlocker;
    FRunnableThread* TimeSyncThread = nullptr;

    bool EnsureRuntimeObjects();
    void PreDeclareAllPDUs();
    bool StartTimeSyncWorker();
    void StopTimeSyncWorker();
};
