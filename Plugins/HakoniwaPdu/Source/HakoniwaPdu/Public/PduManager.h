// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "CommunicationBuffer.h"
#include "CommunicationService.h"
#include "PduChannelLoader.h"
#include "DataPacket.h"
#include "PduManager.generated.h"

/**
 * 
 */
 // ���S��PduManager����

UCLASS()
class HAKONIWAPDU_API UPduManager : public UObject
{
    GENERATED_BODY()

private:
    UPROPERTY()
    UCommunicationBuffer* CommBuffer;

    // WeakPtr���g�p���ďz�Q�Ƃ�h��
    UPROPERTY()
    TWeakObjectPtr<UCommunicationService> CommService;

    UPROPERTY()
    FPduChannelConfigStruct PduConfig;

    // �������t���O
    bool bIsInitialized;

    // �T�[�r�X��Ԃ̃L���b�V��
    bool bLastKnownServiceState;

public:
    UPduManager()
        : CommBuffer(nullptr)
        , bIsInitialized(false)
        , bLastKnownServiceState(false)
    {
    }
    bool DeclarePduForRead(const FString & RobotName, const FString & PduName)
    {
        return DeclarePdu(RobotName, PduName, true);
    }
    bool DeclarePduForWrite(const FString & RobotName, const FString & PduName)
    {
        return DeclarePdu(RobotName, PduName, false);
    }
    bool DeclarePduForReadWrite(const FString & RobotName, const FString & PduName)
    {
        if (DeclarePduForRead(RobotName, PduName)) {
            return DeclarePduForWrite(RobotName, PduName);
        }
        return false;
    }
    void Initialize(const FString& RelativePath, UCommunicationService* InCommService)
    {
        // �����̌���
        if (!IsValid(InCommService))
        {
            UE_LOG(LogTemp, Error, TEXT("UPduManager::Initialize - Invalid CommService provided"));
            return;
        }

        PduConfig = UPduChannelLoader::Load(RelativePath);

        CommBuffer = NewObject<UCommunicationBuffer>(this); // this��e�Ƃ��Đݒ�
        if (!IsValid(CommBuffer))
        {
            UE_LOG(LogTemp, Error, TEXT("UPduManager::Initialize - Failed to create CommBuffer"));
            return;
        }

        CommBuffer->SetChannelConfig(PduConfig);
        CommService = InCommService; // WeakPtr�ɐݒ�
        bIsInitialized = true;

        UE_LOG(LogTemp, Log, TEXT("UPduManager initialized successfully"));
    }

    // ���S�ȃT�[�r�X��ԃ`�F�b�N
    bool IsServiceEnabled()
    {
        // �������`�F�b�N
        if (!bIsInitialized)
        {
            return false;
        }

        // WeakPtr�̗L�����`�F�b�N
        if (!CommService.IsValid())
        {
            UE_LOG(LogTemp, Warning, TEXT("CommService is no longer valid"));
            bLastKnownServiceState = false;
            return false;
        }

        // �ǉ��̈��S���`�F�b�N
        UCommunicationService* Service = CommService.Get();
        if (!IsValid(Service))
        {
            UE_LOG(LogTemp, Warning, TEXT("CommService object is not valid"));
            bLastKnownServiceState = false;
            return false;
        }

        // ���ۂ̃T�[�r�X��Ԃ��`�F�b�N
        bool CurrentState = Service->IsServiceEnabled();
        bLastKnownServiceState = CurrentState;
        return CurrentState;
    }

    bool StartService(const FString& server_uri = TEXT(""))
    {
        if (!bIsInitialized)
        {
            UE_LOG(LogTemp, Error, TEXT("UPduManager not initialized"));
            return false;
        }

        if (!CommService.IsValid())
        {
            UE_LOG(LogTemp, Error, TEXT("CommService is null or invalid"));
            return false;
        }

        UCommunicationService* Service = CommService.Get();
        if (!IsValid(Service))
        {
            UE_LOG(LogTemp, Error, TEXT("CommService object is not valid"));
            return false;
        }

        if (Service->IsServiceEnabled())
        {
            UE_LOG(LogTemp, Warning, TEXT("CommService is already enabled"));
            return false;
        }

        if (!Service->StartService(CommBuffer, server_uri))
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to start CommService"));
            return false;
        }

        bLastKnownServiceState = true;
        return true;
    }

    bool StopService()
    {
        if (!bIsInitialized)
        {
            UE_LOG(LogTemp, Warning, TEXT("UPduManager not initialized"));
            return false;
        }

        if (!CommService.IsValid())
        {
            UE_LOG(LogTemp, Warning, TEXT("CommService is no longer valid"));
            bLastKnownServiceState = false;
            return false;
        }

        UCommunicationService* Service = CommService.Get();
        if (!IsValid(Service))
        {
            UE_LOG(LogTemp, Warning, TEXT("CommService object is not valid"));
            bLastKnownServiceState = false;
            return false;
        }

        if (Service->IsServiceEnabled())
        {
            bool Result = Service->StopService();
            bLastKnownServiceState = !Result;
            return Result;
        }

        bLastKnownServiceState = false;
        return false;
    }

    // ���̑��̃��\�b�h�����S��
    FString GetDefaultRobotName() const
    {
        if (!IsValid(CommBuffer))
        {
            return TEXT("");
        }
        return CommBuffer->GetDefaultRobotName();
    }

    bool IsInitialized() const
    {
        return bIsInitialized;
    }

    TArray<FString> GetRobotNames() const
    {
        if (!IsValid(CommBuffer))
        {
            return {};
        }
        return CommBuffer->GetRobotNames();
    }
    int32 GetPduChannelId(const FString& RobotName, const FString& PduName)
    {
        if (!IsServiceEnabled())
        {
            return -1; // false�ł͂Ȃ��K�؂Ȗ߂�l
        }

        if (!IsValid(CommBuffer))
        {
            UE_LOG(LogTemp, Error, TEXT("CommBuffer is invalid"));
            return -1;
        }

        return CommBuffer->GetPduChannelId(RobotName, PduName);
    }

    int32 GetPduSize(const FString& RobotName, const FString& PduName)
    {
        if (!IsServiceEnabled())
        {
            return -1; // false�ł͂Ȃ��K�؂Ȗ߂�l
        }

        if (!IsValid(CommBuffer))
        {
            UE_LOG(LogTemp, Error, TEXT("CommBuffer is invalid"));
            return -1;
        }

        return CommBuffer->GetPduSize(RobotName, PduName);
    }

    bool FlushPduRawData(const FString& RobotName, const FString& PduName, const TArray<uint8>& PduRawData)
    {
        if (!IsServiceEnabled())
        {
            UE_LOG(LogTemp, Error, TEXT("Service is not enabled"));
            return false;
        }

        if (!IsValid(CommBuffer))
        {
            UE_LOG(LogTemp, Error, TEXT("CommBuffer is invalid"));
            return false;
        }
        int32 ChannelId = CommBuffer->GetPduChannelId(RobotName, PduName);
        if (ChannelId < 0)
        {
            UE_LOG(LogTemp, Error, TEXT("Cannot find Channel ID for %s %s"), *RobotName, *PduName);
            return false;
        }

        // CommService�̍ă`�F�b�N
        if (!CommService.IsValid())
        {
            UE_LOG(LogTemp, Error, TEXT("CommService became invalid during operation"));
            return false;
        }

        UCommunicationService* Service = CommService.Get();
        if (!IsValid(Service))
        {
            UE_LOG(LogTemp, Error, TEXT("CommService object became invalid"));
            return false;
        }

        if (!Service->SendData(RobotName, ChannelId, PduRawData))
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to send data for %s:%s"), *RobotName, *PduName);
            return false;
        }

        return true;
    }
    bool ReadPduRawData(const FString& RobotName, const FString& PduName, TArray<uint8>& OutBuffer)
    {
        if (!IsServiceEnabled() || !IsValid(CommBuffer))
        {
            return false;
        }

        int32 ChannelId = CommBuffer->GetPduChannelId(RobotName, PduName);
        if (ChannelId < 0)
        {
            UE_LOG(LogTemp, Error, TEXT("Cannot find Channel ID for %s %s"), *RobotName, *PduName);
            return false;
        }

        FString Key = CommBuffer->GetKey(RobotName, PduName);
        const TArray<uint8>& RawData = CommBuffer->GetBuffer(Key);

        if (RawData.Num() <= 0)
        {
            return false;
        }

        OutBuffer = RawData;  // ���S�ɃR�s�[ or Move
        return true;
    }

    bool ReadPduRawDataWithMeta(const FString& RobotName, const FString& PduName, TArray<uint8>& OutBuffer, FPduBufferMeta& OutMeta)
    {
        if (!IsServiceEnabled() || !IsValid(CommBuffer))
        {
            OutMeta = FPduBufferMeta();
            return false;
        }

        int32 ChannelId = CommBuffer->GetPduChannelId(RobotName, PduName);
        if (ChannelId < 0)
        {
            UE_LOG(LogTemp, Error, TEXT("Cannot find Channel ID for %s %s"), *RobotName, *PduName);
            OutMeta = FPduBufferMeta();
            return false;
        }

        FString Key = CommBuffer->GetKey(RobotName, PduName);
        return CommBuffer->GetBufferWithMeta(Key, OutBuffer, OutMeta);
    }


    // �f�o�b�O�p�̏�Ԋm�F���\�b�h
    void LogCurrentState() const
    {
        UE_LOG(LogTemp, Log, TEXT("UPduManager State:"));
        UE_LOG(LogTemp, Log, TEXT("  - Initialized: %s"), bIsInitialized ? TEXT("true") : TEXT("false"));
        UE_LOG(LogTemp, Log, TEXT("  - CommBuffer Valid: %s"), IsValid(CommBuffer) ? TEXT("true") : TEXT("false"));
        UE_LOG(LogTemp, Log, TEXT("  - CommService Valid: %s"), CommService.IsValid() ? TEXT("true") : TEXT("false"));
        UE_LOG(LogTemp, Log, TEXT("  - Last Known Service State: %s"), bLastKnownServiceState ? TEXT("true") : TEXT("false"));
    }

    // �N���[���A�b�v
    virtual void BeginDestroy() override
    {
        UE_LOG(LogTemp, Log, TEXT("UPduManager::BeginDestroy called"));

        // �T�[�r�X��~
        if (CommService.IsValid())
        {
            StopService();
        }

        // �Q�Ƃ��N���A
        CommService.Reset();
        CommBuffer = nullptr;
        bIsInitialized = false;

        Super::BeginDestroy();
    }

private:
    bool DeclarePdu(const FString& RobotName, const FString& PduName, bool isRead)
    {
        if (!IsServiceEnabled())
        {
            return false;
        }

        if (!IsValid(CommBuffer))
        {
            UE_LOG(LogTemp, Error, TEXT("CommBuffer is invalid"));
            return false;
        }

        int32 ChannelId = CommBuffer->GetPduChannelId(RobotName, PduName);
        if (ChannelId < 0)
        {
            UE_LOG(LogTemp, Error, TEXT("Cannot find Channel ID for %s %s"), *RobotName, *PduName);
            return false;
        }

        // CommService�̍ă`�F�b�N
        if (!CommService.IsValid())
        {
            UE_LOG(LogTemp, Error, TEXT("CommService became invalid during operation"));
            return false;
        }

        TArray<uint8> PduRawData;
        PduRawData.SetNum(sizeof(uint32));
        uint32 MagicNumber = isRead ? FPduMagicNumbers::DeclarePduForRead : FPduMagicNumbers::DeclarePduForWrite;
        FMemory::Memcpy(PduRawData.GetData(), &MagicNumber, sizeof(uint32));

        UCommunicationService* Service = CommService.Get();
        if (!IsValid(Service))
        {
            UE_LOG(LogTemp, Error, TEXT("CommService object became invalid"));
            return false;
        }

        if (!Service->SendData(RobotName, ChannelId, PduRawData))
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to send magic number for %s:%s"), *RobotName, *PduName);
            return false;
        }

        return true;
    }
};
