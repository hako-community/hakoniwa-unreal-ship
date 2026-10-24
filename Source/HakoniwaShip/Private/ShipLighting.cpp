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
		if (BaseFogDensity < 0.0f)
		{
			BaseFogDensity = Fog->GetComponent()->FogDensity;
		}
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
		// モーションブラーは切る（既定は 0.5。早送りの録画では船・波が大きく動くたびに画面がぶれる）
		Post->Settings.bOverride_MotionBlurAmount = true;
		Post->Settings.MotionBlurAmount = 0.0f;
	}
	if (SkyLight)
	{
		SkyLight->GetLightComponent()->SetCaptureIsDirty();
	}
	UE_LOG(LogShipLighting, Log, TEXT("lighting: %s, %s %.3g lx, exposure EV100 %.1f%s"),
		bNight ? TEXT("night") : TEXT("day"), bNight ? TEXT("moon") : TEXT("sun"), bNight ? MoonLux : SunLux, Ev, Post ? TEXT("") : TEXT(" (no post-process volume)"));
}

void AShipLighting::SetVisibilityM(double VisibilityM)
{
	// 水平に見通すときの霧の透過率（エンジンの HeightFogCommon.ush・SceneCore.cpp）：密度は FogDensity / 1000 [1/cm]、
	// 線積分は密度 × ln2 × 距離 [cm]、透過率 = 2^(−線積分) = exp(−(ln2)²·FogDensity·1e-3·距離 [cm])。
	// 視程（コントラスト 2 %、ln 50 = 3.912）が VisibilityM [m] になる濃さ = 3.912 / ((ln2)²·0.1·VisibilityM) = 81.4 / VisibilityM。
	// 視程が 5 NM（9.26 km）以上は「視界良好」として場面の霞のまま（外乱 PDU の既定 10 km で水平線がかすむと、船橋カメラの水平線の検証が崩れた）。
	// 5 NM 未満では、空の球（100 km 先）にも霧を掛ける（濃霧で青空と雲が見えないように）
	if (!Fog || VisibilityM <= 0.0)
	{
		return;
	}
	if (BaseFogDensity < 0.0f)
	{
		BaseFogDensity = Fog->GetComponent()->FogDensity;
	}
	UExponentialHeightFogComponent* C = Fog->GetComponent();
	if (BaseFogCutoff < 0.0f)
	{
		BaseFogCutoff = C->FogCutoffDistance;
	}
	const bool bRestricted = VisibilityM < 9260.0;
	const float D = bRestricted ? FMath::Max(BaseFogDensity, static_cast<float>(81.4 / FMath::Max(50.0, VisibilityM))) : BaseFogDensity;
	C->SetFogDensity(D);
	C->SetFogCutoffDistance(bRestricted ? 0.0f : BaseFogCutoff);
	UE_LOG(LogTemp, Log, TEXT("ShipLighting: visibility %.0f m -> fog density %.4f, cutoff %.0f"), VisibilityM, D, C->FogCutoffDistance);
}
