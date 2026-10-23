#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "CommunicationService.h"
#include "CommunicationBuffer.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "ShmCommunicationService.generated.h"

class FShmWorker : public FRunnable
{
public:
    FShmWorker(UCommunicationBuffer* InCommBuffer, const FString& InAssetName);
    virtual ~FShmWorker();

    virtual uint32 Run() override;
    virtual void Stop() override;

private:
    UCommunicationBuffer* CommBuffer;
    FString AssetName;
    FThreadSafeBool bStopThread = false;
    uint64 RecvCounter = 0;
    uint64 MotorRecvCounter = 0;
};

/**
 * 
 */
UCLASS()
class HAKONIWAPDU_API UShmCommunicationService : public UCommunicationService
{
	GENERATED_BODY()

private:
    UPROPERTY()
    UCommunicationBuffer* CommBuffer;

    FString AssetName;
    FThreadSafeBool bServiceEnabled = false;

    FShmWorker* Worker = nullptr;
    FRunnableThread* Thread = nullptr;

    TSet<FString> CreatedPduChannels;

public:
    virtual bool StartService(UObject* CommBufferObj, const FString& AssetNameOrUri = TEXT("")) override;
    virtual bool StopService() override;
    virtual bool IsServiceEnabled() const override;
    virtual bool SendData(const FString& RobotName, int32 ChannelId, const TArray<uint8>& PduData) override;
    virtual FString GetServerUri() const override;
};
