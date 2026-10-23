// ShipLighting.cpp

#include "ShipLighting.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMeshActor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "ShipFrame.h"

DEFINE_LOG_CATEGORY_STATIC(LogShipLighting, Log, All);

AShipLighting::AShipLighting()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Lighting"));
}

bool AShipLighting::IsNight()
{
	return FParse::Param(FCommandLine::Get(), TEXT("ShipNight"));
}

void AShipLighting::BeginPlay()
{
	Super::BeginPlay();
	bNight = IsNight();
	Apply();
}

void AShipLighting::Apply()
{
	// 平行光の向き：光は光源から来る（ENU の光源の向きの逆を Unreal へ）
	auto SetLight = [](ADirectionalLight* L, const FVector& DirEnu, float Lux, const FLinearColor& Color, bool bOn)
	{
		if (!L)
		{
			return;
		}
		const FVector D = DirEnu.GetSafeNormal();
		L->SetActorRotation((-ShipFrame::LocalDirToUnreal(D.X, D.Y, D.Z)).Rotation());
		UDirectionalLightComponent* C = CastChecked<UDirectionalLightComponent>(L->GetLightComponent());
		C->SetIntensity(bOn ? Lux : 0.0f);
		C->SetLightColor(Color);
		C->SetVisibility(bOn);
		C->SetCastShadows(bOn);
	};
	SetLight(Sun, SunDirEnu, SunLux, SunColor, !bNight);
	SetLight(Moon, MoonDirEnu, MoonLux, MoonColor, bNight);

	if (SkySphere)
	{
		UStaticMeshComponent* C = SkySphere->GetStaticMeshComponent();
		if (!SkyMid)
		{
			SkyMid = C->CreateAndSetMaterialInstanceDynamic(0);
		}
		if (SkyMid)
		{
			const FVector D = LightDirEnu();
			const FVector U = ShipFrame::LocalDirToUnreal(D.X, D.Y, D.Z);
			SkyMid->SetScalarParameterValue(TEXT("SkyNits"), bNight ? NightSkyNits : SkyNits);
			SkyMid->SetVectorParameterValue(TEXT("Tint"), bNight ? NightTint : FLinearColor::White);
			SkyMid->SetVectorParameterValue(TEXT("DiskDir"), FLinearColor(U.X, U.Y, U.Z));
			SkyMid->SetScalarParameterValue(TEXT("DiskCos"), FMath::Cos(FMath::DegreesToRadians(0.265)));   // 太陽・月の視半径
			SkyMid->SetScalarParameterValue(TEXT("DiskNits"), bNight ? MoonDiskNits : SunDiskNits);
			SkyMid->SetVectorParameterValue(TEXT("DiskColor"), bNight ? FLinearColor(0.9f, 0.92f, 1.0f) : SunColor);
			SkyMid->SetScalarParameterValue(TEXT("Stars"), bNight ? StarNits : 0.0f);
		}
	}
	if (Fog)
	{
		const float Scale = bNight ? NightSkyNits / SkyNits : 1.0f;
		Fog->GetComponent()->SetFogInscatteringColor(bNight ? FogColor * Scale * NightTint : FogColor);
	}
	float Ev = bNight ? NightEV100 : DayEV100;
	FParse::Value(FCommandLine::Get(), TEXT("-ShipExposure="), Ev);   // 調整用の上書き
	if (Post)
	{
		Post->Settings.bOverride_AutoExposureMinBrightness = true;
		Post->Settings.bOverride_AutoExposureMaxBrightness = true;
		Post->Settings.AutoExposureMinBrightness = Ev;
		Post->Settings.AutoExposureMaxBrightness = Ev;
	}
	if (SkyLight)
	{
		SkyLight->GetLightComponent()->SetCaptureIsDirty();
	}
	UE_LOG(LogShipLighting, Log, TEXT("lighting: %s, %s %.3g lx, exposure EV100 %.1f%s"),
		bNight ? TEXT("night") : TEXT("day"), bNight ? TEXT("moon") : TEXT("sun"), bNight ? MoonLux : SunLux, Ev, Post ? TEXT("") : TEXT(" (no post-process volume)"));
}
