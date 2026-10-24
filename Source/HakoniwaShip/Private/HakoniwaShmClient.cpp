// HakoniwaShmClient.cpp
// hakoniwa-unreal-drone（Source/HakoniwaDrone、df4fb2e）の AHakoniwaShmClient をそのまま写したもの（K53）。
// 箱庭にアセットとして登録し、描画とは別のスレッド（FHakoniwaTimeSyncWorker）から世界の時刻に追従して時刻を報告する。
// 描画が遅れても世界の時刻は止まらない。drone から変えたのは、HAKO_CONFIG_PATH が既にあれば上書きしないことだけ。
#include "HakoniwaShmClient.h"
#include <atomic>
#include "Framework/Application/IInputProcessor.h"
#include "Framework/Application/SlateApplication.h"
#include "Modules/ModuleManager.h"
#include "HakoCapiCompat.h"   // ★ Win64 だけ hako_capi.h（共有メモリ）・Android は代用品（A1）
#include "HakoniwaObjectInterface.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

class FHakoniwaTimeSyncWorker final : public FRunnable
{
public:
    FHakoniwaTimeSyncWorker(
        const FString& InAssetName,
        bool bInEnableRealTimePacing,
        double InTargetRealTimeFactor,
        int32 InIntervalMsec)
        : AssetName(InAssetName)
        , bEnableRealTimePacing(bInEnableRealTimePacing)
        , TargetRealTimeFactor(FMath::Max(0.01, InTargetRealTimeFactor))
        , IntervalSec(FMath::Clamp(InIntervalMsec, 1, 20) / 1000.0)
    {
    }

    virtual uint32 Run() override
    {
        UE_LOG(LogTemp, Log, TEXT("[HakoRuntime][TimeSync] asset=%s worker_started=1 pacing=%d target_rtf=%.3f interval_ms=%.3f"),
            *AssetName, bEnableRealTimePacing ? 1 : 0, TargetRealTimeFactor, IntervalSec * 1000.0);

        while (!bStopRequested)
        {
            const double LoopStartWallTime = FPlatformTime::Seconds();
            ProcessOnce(LoopStartWallTime);

            const double WorkSec = FPlatformTime::Seconds() - LoopStartWallTime;
            const float SleepSec = static_cast<float>(FMath::Max(0.0, IntervalSec - WorkSec));
            if (SleepSec > 0.0f)
            {
                FPlatformProcess::SleepNoStats(SleepSec);
            }
        }

        UE_LOG(LogTemp, Log, TEXT("[HakoRuntime][TimeSync] asset=%s worker_stopped=1"), *AssetName);
        return 0;
    }

    virtual void Stop() override
    {
        bStopRequested = true;
    }

    /** 実時間に対する速さを途中で変える（デモの録画：平常は早送り・見せ場は等速）。次のループで今のアセット時刻から測り直す */
    void SetTargetRealTimeFactor(double Factor)
    {
        PendingRtf.store(FMath::Max(0.01, Factor));
    }

    /** アセット時刻の上限 [usec]（録画：表示が追いつくまで物理を先へ進めない）。負なら上限なし */
    void SetTimeCeilingUsec(long long Usec)
    {
        TimeCeilingUsec.store(Usec);
    }

private:
    void ResetPacing()
    {
        bPacingInitialized = false;
        bRtfSampleValid = false;
    }

    void ProcessOnce(double Now)
    {
        const double NewRtf = PendingRtf.exchange(-1.0);
        if (NewRtf > 0.0 && NewRtf != TargetRealTimeFactor)
        {
            TargetRealTimeFactor = NewRtf;
            if (bPacingInitialized)
            {
                PacingBaseWallTime = Now;
                PacingBaseSimTimeUsec = AssetTimeUsec;
            }
            UE_LOG(LogTemp, Log, TEXT("[HakoRuntime][TimeSync] asset=%s target_rtf=%.3f at asset_time_usec=%lld"), *AssetName, TargetRealTimeFactor, AssetTimeUsec);
        }
        const double LoopGapSec = LastLoopWallTime > 0.0 ? Now - LastLoopWallTime : 0.0;
        LastLoopWallTime = Now;

        const int Event = hako_asset_get_event(TCHAR_TO_ANSI(*AssetName));
        switch (Event)
        {
        case 1: // HakoSimAssetEvent_Start
            AssetTimeUsec = 0;
            bAwaitingStart = false;
            ResetPacing();
            UE_LOG(LogTemp, Log, TEXT("Hako Event: START"));
            hako_asset_start_feedback(TCHAR_TO_ANSI(*AssetName), true);
            break;
        case 2: // HakoSimAssetEvent_Stop
            bAwaitingStart = true;
            ResetPacing();
            UE_LOG(LogTemp, Log, TEXT("Hako Event: STOP"));
            hako_asset_stop_feedback(TCHAR_TO_ANSI(*AssetName), true);
            break;
        case 3: // HakoSimAssetEvent_Reset
            AssetTimeUsec = 0;
            bAwaitingStart = true;
            ResetPacing();
            UE_LOG(LogTemp, Log, TEXT("Hako Event: RESET"));
            hako_asset_reset_feedback(TCHAR_TO_ANSI(*AssetName), true);
            break;
        default:
            break;
        }

        const int SimState = hako_simevent_get_state();
        const bool bPduCreated = hako_asset_is_pdu_created();
        const bool bPduSyncMode = hako_asset_is_pdu_sync_mode(TCHAR_TO_ANSI(*AssetName));
        const bool bSimulationMode = hako_asset_is_simulation_mode();
        const long long WorldTimeUsec = static_cast<long long>(hako_asset_get_worldtime());
        const bool bHeartbeatDue = (Now - LastSimtimeNotifyWallTime) >= 1.0;
        bool bNotifySimtime = false;
        bool bWritePduDone = false;
        long long PacingTargetUsec = AssetTimeUsec;
        long long PreNotifyLagUsec = WorldTimeUsec >= 0 ? WorldTimeUsec - AssetTimeUsec : 0;

        if (SimState != 2 /* Running */)
        {
            bNotifySimtime = (SimState == 0 || SimState == 1) && bHeartbeatDue;
            ResetPacing();
        }
        else if (bPduCreated && !bAwaitingStart)
        {
            if (bPduSyncMode)
            {
                hako_asset_notify_write_pdu_done(TCHAR_TO_ANSI(*AssetName));
                bWritePduDone = true;
                ResetPacing();
            }
            else if (bSimulationMode && WorldTimeUsec >= 0)
            {
                // A debugger break or machine sleep must not cause an unbounded
                // fast-forward toward wall time when this process resumes.
                const bool bLongWorkerPause = LoopGapSec > FMath::Max(0.25, IntervalSec * 10.0);
                if (!bPacingInitialized || WorldTimeUsec < LastObservedWorldTimeUsec || bLongWorkerPause)
                {
                    PacingBaseWallTime = Now;
                    PacingBaseSimTimeUsec = WorldTimeUsec;
                    AssetTimeUsec = WorldTimeUsec;
                    bPacingInitialized = true;
                    bRtfSampleValid = false;
                    if (bLongWorkerPause)
                    {
                        ++PacingRebaseCount;
                    }
                }

                if (bEnableRealTimePacing)
                {
                    const double ElapsedWallSec = FMath::Max(0.0, Now - PacingBaseWallTime);
                    PacingTargetUsec = PacingBaseSimTimeUsec
                        + static_cast<long long>(ElapsedWallSec * TargetRealTimeFactor * 1000000.0);
                    PacingTargetUsec = FMath::Min(PacingTargetUsec, WorldTimeUsec);
                }
                else
                {
                    PacingTargetUsec = WorldTimeUsec;
                }
                if (const long long Ceil = TimeCeilingUsec.load(); Ceil >= 0)
                {
                    PacingTargetUsec = FMath::Min(PacingTargetUsec, Ceil);
                }

                if (PacingTargetUsec > AssetTimeUsec)
                {
                    AssetTimeUsec = PacingTargetUsec;
                    bNotifySimtime = true;
                }
                else if (PacingTargetUsec == AssetTimeUsec && bHeartbeatDue)
                {
                    bNotifySimtime = true;
                }
            }
        }

        if (bNotifySimtime)
        {
            const double NotifyIntervalMs = LastSimtimeNotifyWallTime > 0.0
                ? (Now - LastSimtimeNotifyWallTime) * 1000.0
                : 0.0;
            MaxNotifyIntervalMs = FMath::Max(MaxNotifyIntervalMs, NotifyIntervalMs);
            ++NotifyCount;
            hako_asset_notify_simtime(TCHAR_TO_ANSI(*AssetName), AssetTimeUsec);
            LastSimtimeNotifyWallTime = Now;
        }

        const bool bCanMeasureRtf = SimState == 2 && bPduCreated && bSimulationMode
            && !bPduSyncMode && !bAwaitingStart && WorldTimeUsec >= 0;
        if (!bCanMeasureRtf || WorldTimeUsec < LastObservedWorldTimeUsec)
        {
            bRtfSampleValid = false;
        }
        LastObservedWorldTimeUsec = WorldTimeUsec;

        if ((Now - LastRuntimeStateLogTime) >= 1.0)
        {
            const double WallDeltaSec = LastRuntimeStateLogTime > 0.0 ? Now - LastRuntimeStateLogTime : 0.0;
            const bool bRtfValid = bRtfSampleValid && bCanMeasureRtf && WallDeltaSec > 0.0;
            const long long WorldDeltaUsec = bRtfValid ? WorldTimeUsec - LastLoggedWorldTimeUsec : 0;
            const double Rtf = bRtfValid ? static_cast<double>(WorldDeltaUsec) / (WallDeltaSec * 1000000.0) : 0.0;
            const long long LagUsec = WorldTimeUsec >= 0 ? WorldTimeUsec - AssetTimeUsec : 0;
            UE_LOG(LogTemp, Log, TEXT("[HakoRuntime][State] asset=%s sim_state=%d pdu_created=%d sim_mode=%d pdu_sync_mode=%d world_time_usec=%lld asset_time_usec=%lld lag_usec=%lld pre_notify_lag_usec=%lld pacing_target_usec=%lld world_delta_usec=%lld wall_time_sec=%.6f wall_delta_sec=%.6f rtf=%.6f rtf_valid=%d world_time_valid=%d simtime_notified=%d write_pdu_done=%d awaiting_start=%d pacing=%d target_rtf=%.3f notify_count=%llu max_notify_interval_ms=%.3f pacing_rebase_count=%llu"),
                *AssetName, SimState, bPduCreated ? 1 : 0, bSimulationMode ? 1 : 0, bPduSyncMode ? 1 : 0,
                WorldTimeUsec, AssetTimeUsec, LagUsec, PreNotifyLagUsec, PacingTargetUsec, WorldDeltaUsec,
                Now, WallDeltaSec, Rtf, bRtfValid ? 1 : 0, WorldTimeUsec >= 0 ? 1 : 0,
                bNotifySimtime ? 1 : 0, bWritePduDone ? 1 : 0, bAwaitingStart ? 1 : 0,
                bEnableRealTimePacing ? 1 : 0, TargetRealTimeFactor,
                static_cast<unsigned long long>(NotifyCount), MaxNotifyIntervalMs,
                static_cast<unsigned long long>(PacingRebaseCount));
            LastRuntimeStateLogTime = Now;
            LastLoggedWorldTimeUsec = WorldTimeUsec;
            bRtfSampleValid = bCanMeasureRtf;
            NotifyCount = 0;
            MaxNotifyIntervalMs = 0.0;
            PacingRebaseCount = 0;
        }
    }

    FString AssetName;
    bool bEnableRealTimePacing = true;
    double TargetRealTimeFactor = 1.0;
    std::atomic<double> PendingRtf {-1.0};
    std::atomic<long long> TimeCeilingUsec {-1};
    double IntervalSec = 0.005;
    FThreadSafeBool bStopRequested = false;
    long long AssetTimeUsec = 0;
    bool bAwaitingStart = false;
    bool bPacingInitialized = false;
    double PacingBaseWallTime = 0.0;
    double LastLoopWallTime = 0.0;
    long long PacingBaseSimTimeUsec = 0;
    double LastSimtimeNotifyWallTime = 0.0;
    double LastRuntimeStateLogTime = 0.0;
    long long LastLoggedWorldTimeUsec = 0;
    long long LastObservedWorldTimeUsec = -1;
    bool bRtfSampleValid = false;
    uint64 NotifyCount = 0;
    uint64 PacingRebaseCount = 0;
    double MaxNotifyIntervalMs = 0.0;
};

AHakoniwaShmClient::AHakoniwaShmClient()
{
    PrimaryActorTick.bCanEverTick = true;
}

namespace
{
/// ★ 2026-10-04: ゲームパッドのキーとスティックを、UI（画面のボタン）と視点に届く前に捨てる。
class FHakoGamepadBlocker : public IInputProcessor
{
public:
    virtual void Tick(const float, FSlateApplication&, TSharedRef<ICursor>) override {}
    virtual bool HandleKeyDownEvent(FSlateApplication&, const FKeyEvent& E) override { return E.GetKey().IsGamepadKey(); }
    virtual bool HandleKeyUpEvent(FSlateApplication&, const FKeyEvent& E) override { return E.GetKey().IsGamepadKey(); }
    virtual bool HandleAnalogInputEvent(FSlateApplication&, const FAnalogInputEvent& E) override { return E.GetKey().IsGamepadKey(); }
    virtual const TCHAR* GetDebugName() const override { return TEXT("HakoGamepadBlocker"); }
};
}

void AHakoniwaShmClient::BeginPlay()
{
    Super::BeginPlay();

    if (bIgnoreGamepad && FSlateApplication::IsInitialized())
    {
        GamepadBlocker = MakeShared<FHakoGamepadBlocker>();
        FSlateApplication::Get().RegisterInputPreProcessor(GamepadBlocker);
        UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient: gamepad input is ignored (bIgnoreGamepad). The external sender (rc_pdu_pub.py) reads the pad."));
    }

    UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient BeginPlay()"));

    if (!EnsureRuntimeObjects())
    {
        UE_LOG(LogTemp, Error, TEXT("AHakoniwaShmClient BeginPlay() - Failed to create runtime objects."));
        return;
    }

    if (bAutoInitialize)
    {
        UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient BeginPlay() - Auto initializing client."));
        if (!InitializeClient())
        {
            UE_LOG(LogTemp, Error, TEXT("AHakoniwaShmClient BeginPlay() - Auto initialization failed."));
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("AHakoniwaShmClient BeginPlay() - Auto initialization is disabled. START UI may stay disabled until InitializeClient is called."));
    }
}

void AHakoniwaShmClient::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (GamepadBlocker.IsValid() && FSlateApplication::IsInitialized())
    {
        FSlateApplication::Get().UnregisterInputPreProcessor(GamepadBlocker);   // ★ エディタの PIE のあとに残さない
    }
    GamepadBlocker.Reset();
    StopTimeSyncWorker();
    if (pduManager && service && service->IsServiceEnabled())
    {
        pduManager->StopService();
    }
    Super::EndPlay(EndPlayReason);
}

bool AHakoniwaShmClient::EnsureRuntimeObjects()
{
    const FName ModuleName = "HakoniwaPdu";
    if (!FModuleManager::Get().IsModuleLoaded(ModuleName))
    {
        FModuleManager::Get().LoadModule(ModuleName);
    }

    if (!service)
    {
        service = NewObject<UShmCommunicationService>(this);
    }

    if (!pduManager)
    {
        pduManager = NewObject<UPduManager>(this);
    }

    return service != nullptr && pduManager != nullptr;
}

bool AHakoniwaShmClient::InitializeClient()
{
    if (!EnsureRuntimeObjects())
    {
        UE_LOG(LogTemp, Error, TEXT("AHakoniwaShmClient::InitializeClient - Failed to create runtime objects."));
        return false;
    }

    if (service && service->IsServiceEnabled())
    {
        UE_LOG(LogTemp, Warning, TEXT("AHakoniwaShmClient::InitializeClient - Already initialized. Skipping."));
        return true;
    }

    UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient InitializeClient() started"));

    // 1. Prepare absolute path to cpp_core_config.json
    FString FullConfigPath = FPaths::ProjectContentDir() / ConfigPath;
    if (!FPaths::FileExists(FullConfigPath))
    {
        UE_LOG(LogTemp, Error, TEXT("ConfigPath not found: %s"), *FullConfigPath);
        return false;
    }
    FString AbsoluteConfigPath = FPaths::ConvertRelativePathToFull(FullConfigPath);
    // Normalize path for the platform
    FPaths::NormalizeFilename(AbsoluteConfigPath);
    AbsoluteConfigPath = FPaths::CreateStandardFilename(AbsoluteConfigPath);

    UE_LOG(LogTemp, Log, TEXT("Using Config Path: %s"), *AbsoluteConfigPath);

    // 2. Initialize Hako Asset. Current shakoc.dll reads this path from HAKO_CONFIG_PATH.
    //    ★ rover：HAKO_CONFIG_PATH（共有メモリの設定。mujoco-rover の run_sim.ps1 が作る）が既にあれば上書きしない
    if (FPlatformMisc::GetEnvironmentVariable(TEXT("HAKO_CONFIG_PATH")).IsEmpty())
    {
        FPlatformMisc::SetEnvironmentVar(TEXT("HAKO_CONFIG_PATH"), *AbsoluteConfigPath);
    }
    UE_LOG(LogTemp, Log, TEXT("Calling hako_asset_init with HAKO_CONFIG_PATH=%s"), *FPlatformMisc::GetEnvironmentVariable(TEXT("HAKO_CONFIG_PATH")));
    if (!hako_asset_init())
    {
        UE_LOG(LogTemp, Error, TEXT("hako_asset_init FAILED. Check HAKO_CONFIG_PATH and core mmap files."));
        return false;
    }

    if (!pduManager)
    {
        UE_LOG(LogTemp, Error, TEXT("pduManager is null."));
        return false;
    }

    // 3. Initialize UPduManager (This loads the PDU config from JSON)
    pduManager->Initialize(ConfigPath, service);

    // 4. Start Service (This will do registration)
    UE_LOG(LogTemp, Log, TEXT("Calling pduManager->StartService for asset: %s"), *AssetName);
    if (!pduManager->StartService(AssetName))
    {
        UE_LOG(LogTemp, Error, TEXT("pduManager->StartService failed."));
        return false;
    }

    // 5. Pre-declare PDUs by notifying all Hakoniwa objects
    UE_LOG(LogTemp, Log, TEXT("Notifying all Hakoniwa objects for PDU declaration..."));
    PreDeclareAllPDUs();

    if (!StartTimeSyncWorker())
    {
        UE_LOG(LogTemp, Error, TEXT("AHakoniwaShmClient::InitializeClient - Failed to start time sync worker."));
        pduManager->StopService();
        return false;
    }

    UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient initialized successfully."));
    return true;
}

void AHakoniwaShmClient::PreDeclareAllPDUs()
{
    TArray<AActor*> Actors;
    UGameplayStatics::GetAllActorsWithInterface(GetWorld(), UHakoniwaObjectInterface::StaticClass(), Actors);
    for (AActor* Actor : Actors)
    {
        UE_LOG(LogTemp, Log, TEXT("Notifying OnHakoInitialize to %s"), *Actor->GetName());
        IHakoniwaObjectInterface::Execute_OnHakoInitialize(Actor);
    }
}

void AHakoniwaShmClient::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void AHakoniwaShmClient::SetTimeCeiling(double Sec)
{
    if (TimeSyncWorker)
    {
        TimeSyncWorker->SetTimeCeilingUsec(Sec < 0.0 ? -1 : static_cast<long long>(Sec * 1000000.0));
    }
}

void AHakoniwaShmClient::SetRealTimeFactor(float Factor)
{
    TargetRealTimeFactor = Factor;
    if (TimeSyncWorker)
    {
        TimeSyncWorker->SetTargetRealTimeFactor(Factor);
    }
}

bool AHakoniwaShmClient::StartTimeSyncWorker()
{
    if (TimeSyncThread || TimeSyncWorker)
    {
        return true;
    }

    TimeSyncWorker = new FHakoniwaTimeSyncWorker(
        AssetName,
        bEnableRealTimePacing,
        static_cast<double>(TargetRealTimeFactor),
        TimeSyncIntervalMsec);
    TimeSyncThread = FRunnableThread::Create(TimeSyncWorker, TEXT("HakoniwaTimeSyncWorker"));
    if (!TimeSyncThread)
    {
        delete TimeSyncWorker;
        TimeSyncWorker = nullptr;
        return false;
    }
    return true;
}

void AHakoniwaShmClient::StopTimeSyncWorker()
{
    if (TimeSyncWorker)
    {
        TimeSyncWorker->Stop();
    }
    if (TimeSyncThread)
    {
        TimeSyncThread->WaitForCompletion();
        delete TimeSyncThread;
        TimeSyncThread = nullptr;
    }
    if (TimeSyncWorker)
    {
        delete TimeSyncWorker;
        TimeSyncWorker = nullptr;
    }
}

void AHakoniwaShmClient::Start_Implementation()
{
    if (!service || !service->IsServiceEnabled())
    {
        UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient Start() - Initializing client before start."));
        if (!InitializeClient())
        {
            UE_LOG(LogTemp, Error, TEXT("AHakoniwaShmClient Start() - InitializeClient failed. Start canceled."));
            return;
        }
    }

    UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient Start() - Sending simevent_start"));
    hako_simevent_start();
}

void AHakoniwaShmClient::Stop_Implementation()
{
    if (!service || !service->IsServiceEnabled())
    {
        UE_LOG(LogTemp, Warning, TEXT("AHakoniwaShmClient Stop() - Client is not initialized."));
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient Stop() - Sending simevent_stop"));
    hako_simevent_stop();
}

void AHakoniwaShmClient::Reset_Implementation()
{
    if (!service || !service->IsServiceEnabled())
    {
        UE_LOG(LogTemp, Warning, TEXT("AHakoniwaShmClient Reset() - Client is not initialized."));
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("AHakoniwaShmClient Reset() - Sending simevent_reset"));
    hako_simevent_reset();
}

int32 AHakoniwaShmClient::GetSimulationState_Implementation() const
{
    if (!service || !service->IsServiceEnabled())
    {
        return 0; // Treat uninitialized as stopped so the UI can trigger Start().
    }
    return hako_simevent_get_state();
}
