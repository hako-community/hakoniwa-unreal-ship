#include "ShmCommunicationService.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/PlatformTLS.h"
#include "HakoCapiCompat.h"   // ★ Win64 だけ hako_capi.h（共有メモリ）・Android は代用品（A1）

#ifndef HAKO_PROP_DETAILED_LOG
#define HAKO_PROP_DETAILED_LOG 0
#endif

static TAutoConsoleVariable<float> CVarHakoPduShmPollSleepSec(
    TEXT("hako.Pdu.ShmPollSleepSec"),
    0.01f,
    TEXT("Sleep interval in seconds for Hakoniwa SHM PDU polling worker. Default is 0.01."),
    ECVF_Default);

FShmWorker::FShmWorker(UCommunicationBuffer* InCommBuffer, const FString& InAssetName)
    : CommBuffer(InCommBuffer), AssetName(InAssetName)
{
}

FShmWorker::~FShmWorker()
{
}

uint32 FShmWorker::Run()
{
    UE_LOG(LogTemp, Log, TEXT("FShmWorker started for asset: %s"), *AssetName);
    float CurrentSleepSec = FMath::Max(0.0f, CVarHakoPduShmPollSleepSec.GetValueOnAnyThread());
#if HAKO_PDU_LATENCY_LOG
    UE_LOG(LogTemp, Log, TEXT("[HakoPDU][Config] shm_poll_sleep_sec=%.3f"), CurrentSleepSec);
#endif

    double LastPollTime = FPlatformTime::Seconds();
    double PollStatsStartTime = LastPollTime;
    double PollIntervalSumMs = 0.0;
    double PollIntervalMinMs = TNumericLimits<double>::Max();
    double PollIntervalMaxMs = 0.0;
    int32 PollSampleCount = 0;
    int32 PollDirtyCount = 0;
    int32 PollReadCount = 0;
    int32 MotorDirtyCount = 0;
    int32 MotorReadCount = 0;
    int32 MotorReadFailCount = 0;
    int32 DeclaredReaderCount = 0;
    int32 CheckedReaderCount = 0;
    uint64 LatestMotorCounter = 0;
    int32 LatestMotorSize = 0;
    FString DeclaredReaderSummary;
    bool bLoggedReaderSummary = false;

    while (!bStopThread)
    {
        CurrentSleepSec = FMath::Max(0.0f, CVarHakoPduShmPollSleepSec.GetValueOnAnyThread());
        const double PollStartTime = FPlatformTime::Seconds();
        const double PollIntervalMs = (PollStartTime - LastPollTime) * 1000.0;
        LastPollTime = PollStartTime;
        PollIntervalSumMs += PollIntervalMs;
        PollIntervalMinMs = FMath::Min(PollIntervalMinMs, PollIntervalMs);
        PollIntervalMaxMs = FMath::Max(PollIntervalMaxMs, PollIntervalMs);
        ++PollSampleCount;
        bool bPosDirtyThisPoll = false;
        bool bPosReadThisPoll = false;
        bool bMotorDirtyThisPoll = false;
        bool bMotorReadThisPoll = false;
        bool bMotorReadFailedThisPoll = false;

        if (CommBuffer)
        {
            const FPduChannelConfigStruct& ChannelConfig = CommBuffer->GetChannelConfig();
            bool bIsUpdated = false;
            if (!bLoggedReaderSummary)
            {
                int32 ConfigReaderCount = 0;
                for (const FRobotConfig& Robot : ChannelConfig.robots)
                {
                    for (const FPduChannel& Ch : Robot.shm_pdu_readers)
                    {
                        ++ConfigReaderCount;
                        DeclaredReaderSummary += FString::Printf(TEXT("%s:%s(ch=%d,size=%d,decl=%d); "),
                            *Robot.name,
                            *Ch.org_name,
                            Ch.channel_id,
                            Ch.pdu_size,
                            CommBuffer->IsPduDeclaredForRead(Robot.name, Ch.channel_id) ? 1 : 0);
                    }
                }
                UE_LOG(LogTemp, Log, TEXT("[HakoPDU][ShmReaders] asset=%s configured=%d readers=%s"),
                    *AssetName,
                    ConfigReaderCount,
                    *DeclaredReaderSummary);
                bLoggedReaderSummary = true;
            }

            for (const FRobotConfig& Robot : ChannelConfig.robots)
            {
                for (const FPduChannel& Ch : Robot.shm_pdu_readers)
                {
                    if (!CommBuffer->IsPduDeclaredForRead(Robot.name, Ch.channel_id))
                    {
                        continue;
                    }
                    ++DeclaredReaderCount;
                    ++CheckedReaderCount;
                    if (hako_asset_is_pdu_dirty(TCHAR_TO_ANSI(*AssetName), TCHAR_TO_ANSI(*Robot.name), Ch.channel_id))
                    {
                        TArray<uint8> PduData;
                        PduData.SetNumUninitialized(Ch.pdu_size);
                        if (hako_asset_read_pdu(TCHAR_TO_ANSI(*AssetName), TCHAR_TO_ANSI(*Robot.name), Ch.channel_id, (char*)PduData.GetData(), Ch.pdu_size))
                        {
                            if (Ch.org_name == TEXT("pos"))
                            {
                                bPosDirtyThisPoll = true;
                                bPosReadThisPoll = true;

                                FPduBufferMeta Meta;
                                Meta.RecvCounter = ++RecvCounter;
                                Meta.RecvUeTime = FPlatformTime::Seconds();
                                Meta.DataSize = PduData.Num();
                                CommBuffer->PutPacketDirectWithMeta(Robot.name, Ch.channel_id, PduData, Meta);

#if HAKO_PDU_LATENCY_LOG
                                UE_LOG(LogTemp, Log, TEXT("[HakoPDU][Recv] asset=%s robot=%s pdu=%s counter=%llu ue_time=%.6f size=%d thread=%u"),
                                    *AssetName,
                                    *Robot.name,
                                    *Ch.org_name,
                                    static_cast<unsigned long long>(Meta.RecvCounter),
                                    Meta.RecvUeTime,
                                    Meta.DataSize,
                                    FPlatformTLS::GetCurrentThreadId());
#endif
                            }
                            else if (Ch.org_name == TEXT("motor"))
                            {
                                bMotorDirtyThisPoll = true;
                                bMotorReadThisPoll = true;

                                FPduBufferMeta Meta;
                                Meta.RecvCounter = ++MotorRecvCounter;
                                Meta.RecvUeTime = FPlatformTime::Seconds();
                                Meta.DataSize = PduData.Num();
                                LatestMotorCounter = Meta.RecvCounter;
                                LatestMotorSize = Meta.DataSize;
                                CommBuffer->PutPacketDirectWithMeta(Robot.name, Ch.channel_id, PduData, Meta);

#if HAKO_PROP_STATS_LOG && HAKO_PROP_DETAILED_LOG
                                UE_LOG(LogTemp, Log, TEXT("[HakoProp][Recv] asset=%s robot=%s pdu=%s counter=%llu recv=%.6f size=%d thread=%u"),
                                    *AssetName,
                                    *Robot.name,
                                    *Ch.org_name,
                                    static_cast<unsigned long long>(Meta.RecvCounter),
                                    Meta.RecvUeTime,
                                    Meta.DataSize,
                                    FPlatformTLS::GetCurrentThreadId());
#endif
                            }
                            else
                            {
                                CommBuffer->PutPacketDirect(Robot.name, Ch.channel_id, PduData);
                            }
                            bIsUpdated = true;
                        }
                        else if (Ch.org_name == TEXT("pos"))
                        {
                            bPosDirtyThisPoll = true;
                        }
                        else if (Ch.org_name == TEXT("motor"))
                        {
                            bMotorDirtyThisPoll = true;
                            bMotorReadFailedThisPoll = true;
                        }
                    }
                }
            }

            if (bIsUpdated)
            {
                hako_asset_notify_read_pdu_done(TCHAR_TO_ANSI(*AssetName));
            }
        }

        if (bPosDirtyThisPoll)
        {
            ++PollDirtyCount;
        }
        if (bPosReadThisPoll)
        {
            ++PollReadCount;
        }
        if (bMotorDirtyThisPoll)
        {
            ++MotorDirtyCount;
        }
        if (bMotorReadThisPoll)
        {
            ++MotorReadCount;
        }
        if (bMotorReadFailedThisPoll)
        {
            ++MotorReadFailCount;
        }

#if HAKO_PDU_LATENCY_LOG || HAKO_PROP_STATS_LOG
        const double PollNow = FPlatformTime::Seconds();
        if ((PollNow - PollStatsStartTime) >= 1.0)
        {
            const double AvgPollMs = PollSampleCount > 0 ? PollIntervalSumMs / static_cast<double>(PollSampleCount) : 0.0;
            const double MinPollMs = PollSampleCount > 0 ? PollIntervalMinMs : 0.0;
#if HAKO_PDU_LATENCY_LOG
            UE_LOG(LogTemp, Log, TEXT("[HakoPDU][Poll] asset=%s sleep=%.3f interval_avg_ms=%.3f interval_min_ms=%.3f interval_max_ms=%.3f samples=%d dirty=%d read=%d thread=%u"),
                *AssetName,
                CurrentSleepSec,
                AvgPollMs,
                MinPollMs,
                PollIntervalMaxMs,
                PollSampleCount,
                PollDirtyCount,
                PollReadCount,
                FPlatformTLS::GetCurrentThreadId());
#endif
#if HAKO_PROP_STATS_LOG
            UE_LOG(LogTemp, Log, TEXT("[HakoProp][RecvStats] asset=%s sleep=%.3f samples=%d declared_checks=%d reader_checks=%d dirty=%d read=%d read_fail=%d latest_counter=%llu latest_size=%d thread=%u"),
                *AssetName,
                CurrentSleepSec,
                PollSampleCount,
                DeclaredReaderCount,
                CheckedReaderCount,
                MotorDirtyCount,
                MotorReadCount,
                MotorReadFailCount,
                static_cast<unsigned long long>(LatestMotorCounter),
                LatestMotorSize,
                FPlatformTLS::GetCurrentThreadId());
#endif

            PollStatsStartTime = PollNow;
            PollIntervalSumMs = 0.0;
            PollIntervalMinMs = TNumericLimits<double>::Max();
            PollIntervalMaxMs = 0.0;
            PollSampleCount = 0;
            PollDirtyCount = 0;
            PollReadCount = 0;
            MotorDirtyCount = 0;
            MotorReadCount = 0;
            MotorReadFailCount = 0;
            DeclaredReaderCount = 0;
            CheckedReaderCount = 0;
        }
#endif

        FPlatformProcess::Sleep(CurrentSleepSec);
    }

    return 0;
}

void FShmWorker::Stop()
{
    bStopThread = true;
}

bool UShmCommunicationService::StartService(UObject* CommBufferObj, const FString& InAssetName)
{
    UE_LOG(LogTemp, Log, TEXT("UShmCommunicationService::StartService - Started for asset: %s"), *InAssetName);
    CommBuffer = Cast<UCommunicationBuffer>(CommBufferObj);
    AssetName = InAssetName;

    if (!CommBuffer)
    {
        UE_LOG(LogTemp, Error, TEXT("UShmCommunicationService::StartService - CommBuffer is invalid"));
        return false;
    }

    UE_LOG(LogTemp, Log, TEXT("UShmCommunicationService::StartService - Calling hako_asset_register_polling(%s)"), *AssetName);
    bool bRegSuccess = hako_asset_register_polling(TCHAR_TO_ANSI(*AssetName));
    if (!bRegSuccess)
    {
        UE_LOG(LogTemp, Error, TEXT("UShmCommunicationService::StartService - hako_asset_register_polling FAILED for %s"), *AssetName);
        return false;
    }
    UE_LOG(LogTemp, Log, TEXT("UShmCommunicationService::StartService - hako_asset_register_polling SUCCEEDED"));

    bServiceEnabled = true;
    CreatedPduChannels.Empty();
    UE_LOG(LogTemp, Log, TEXT("UShmCommunicationService::StartService - PDU LChannels will be created lazily from declarations/writes"));

    Worker = new FShmWorker(CommBuffer, AssetName);
    Thread = FRunnableThread::Create(Worker, TEXT("HakoniwaShmWorker"));

    UE_LOG(LogTemp, Log, TEXT("UShmCommunicationService::StartService - Thread created successfully"));
    return true;
}

bool UShmCommunicationService::StopService()
{
    bServiceEnabled = false;

    if (Thread)
    {
        Thread->Kill(true);
        delete Thread;
        Thread = nullptr;
    }

    if (Worker)
    {
        delete Worker;
        Worker = nullptr;
    }

    hako_asset_unregister(TCHAR_TO_ANSI(*AssetName));

    return true;
}

bool UShmCommunicationService::IsServiceEnabled() const
{
    return bServiceEnabled;
}

bool UShmCommunicationService::SendData(const FString& RobotName, int32 ChannelId, const TArray<uint8>& PduData)
{
    if (!bServiceEnabled) return false;

    // Declaration packets are local subscription metadata for this Unreal-side
    // service. Do not create Hakoniwa lchannels here; the SHM channels are owned
    // by the external simulator/input pipeline in visualizer-only mode.
    if (PduData.Num() == 4)
    {
        uint32 Magic;
        FMemory::Memcpy(&Magic, PduData.GetData(), 4);
        if (Magic == FPduMagicNumbers::DeclarePduForRead || Magic == FPduMagicNumbers::DeclarePduForWrite)
        {
            FString PduName = CommBuffer != nullptr ? CommBuffer->GetPduName(RobotName, ChannelId) : TEXT("");
            if (Magic == FPduMagicNumbers::DeclarePduForRead && CommBuffer != nullptr)
            {
                CommBuffer->MarkPduDeclaredForRead(RobotName, ChannelId);
                UE_LOG(LogTemp, Log, TEXT("UShmCommunicationService: registered read declaration robot=%s channel=%d pdu=%s"),
                    *RobotName,
                    ChannelId,
                    *PduName);
            }
            else
            {
                UE_LOG(LogTemp, Log, TEXT("UShmCommunicationService: registered write declaration robot=%s channel=%d pdu=%s"),
                    *RobotName,
                    ChannelId,
                    *PduName);
            }
            return true;
        }
    }

    FString Key = RobotName + FString::FromInt(ChannelId);
    if (!CreatedPduChannels.Contains(Key))
    {
        int32 PduSize = PduData.Num();

        if (hako_asset_create_pdu_lchannel(TCHAR_TO_ANSI(*RobotName), ChannelId, PduSize))
        {
            CreatedPduChannels.Add(Key);
            UE_LOG(LogTemp, Log, TEXT("Created PDU LChannel: %s, %d, size=%d"), *RobotName, ChannelId, PduSize);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create PDU LChannel: %s, %d"), *RobotName, ChannelId);
            return false;
        }
    }

    bool ret = hako_asset_write_pdu(TCHAR_TO_ANSI(*AssetName), TCHAR_TO_ANSI(*RobotName), ChannelId, (const char*)PduData.GetData(), PduData.Num());
    if (ret)
    {
        hako_asset_notify_write_pdu_done(TCHAR_TO_ANSI(*AssetName));
    }
    return ret;
}

FString UShmCommunicationService::GetServerUri() const
{
    return AssetName;
}
