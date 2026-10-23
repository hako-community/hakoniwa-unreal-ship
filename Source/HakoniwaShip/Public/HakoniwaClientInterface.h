#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "PduManager.h"
#include "HakoniwaClientInterface.generated.h"

UINTERFACE(MinimalAPI, Blueprintable)
class UHakoniwaClientInterface : public UInterface
{
	GENERATED_BODY()
};

class HAKONIWASHIP_API IHakoniwaClientInterface
{
	GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Hakoniwa")
	UPduManager* GetPduManager() const;

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Hakoniwa")
    void Start();

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Hakoniwa")
    void Stop();

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Hakoniwa")
    void Reset();

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Hakoniwa")
    int32 GetSimulationState() const;
};
