#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "HakoniwaObjectInterface.generated.h"

UINTERFACE(MinimalAPI, Blueprintable)
class UHakoniwaObjectInterface : public UInterface
{
	GENERATED_BODY()
};

class HAKONIWASHIP_API IHakoniwaObjectInterface
{
	GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Hakoniwa")
    void OnHakoInitialize();

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Hakoniwa")
    void OnHakoReset();
};
