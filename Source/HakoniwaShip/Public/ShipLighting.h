// ShipLighting.h
// 昼と夜の照明（SHP-24 Step 5）。場面づくり（HakoniwaShipEditor）が太陽・月・空の光・空の球・霧・露出の部品とその値を入れて置く。
//
//   昼（既定）：パノラマの太陽（向き・照度は SourceArt/Sky/sky.json）、空の球にパノラマ、太陽の円盤（ブルーム・レンズフレアのまぶしさ）、露出 EV100 15.5
//   -ShipExposure=<EV100> で露出を上書きする（調整用）
//   夜（-ShipNight）：月（満月 0.25 lx、青みの光、影あり）、空はパノラマを暗く青くしたものと星・月の円盤、霧も暗く、露出 EV100 0.5。
//                     航海灯は AShipActor が光源を点ける（射光範囲は材質と光源の向き）
// 場面の時刻は PDU に無いので、ビューアの起動の引数で選ぶ。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ShipLighting.generated.h"

class ADirectionalLight;
class AExponentialHeightFog;
class APostProcessVolume;
class ASkyLight;
class AStaticMeshActor;
class UMaterialInstanceDynamic;

UCLASS()
class HAKONIWASHIP_API AShipLighting : public AActor
{
	GENERATED_BODY()

public:
	AShipLighting();

	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, Category = "Lighting") TObjectPtr<ADirectionalLight> Sun;
	UPROPERTY(EditAnywhere, Category = "Lighting") TObjectPtr<ADirectionalLight> Moon;
	UPROPERTY(EditAnywhere, Category = "Lighting") TObjectPtr<ASkyLight> SkyLight;
	UPROPERTY(EditAnywhere, Category = "Lighting") TObjectPtr<AStaticMeshActor> SkySphere;
	UPROPERTY(EditAnywhere, Category = "Lighting") TObjectPtr<AExponentialHeightFog> Fog;
	UPROPERTY(EditAnywhere, Category = "Lighting") TObjectPtr<APostProcessVolume> Post;

	// 昼（太陽の向きは ENU）
	UPROPERTY(EditAnywhere, Category = "Day") FVector SunDirEnu = FVector(0.0, -0.7, 0.7);
	UPROPERTY(EditAnywhere, Category = "Day") float SunLux = 89000.0f;
	UPROPERTY(EditAnywhere, Category = "Day") FLinearColor SunColor = FLinearColor::White;
	UPROPERTY(EditAnywhere, Category = "Day") float SkyNits = 20000.0f;          // パノラマの値 1 の輝度
	UPROPERTY(EditAnywhere, Category = "Day") FLinearColor FogColor = FLinearColor(9000.0f, 9000.0f, 11000.0f);
	UPROPERTY(EditAnywhere, Category = "Day") float DayEV100 = 15.5f;
	UPROPERTY(EditAnywhere, Category = "Day") float SunDiskNits = 2.0e6f;        // 太陽の円盤（実際は約 1e9。半精度の画像があふれないよう抑える）

	// 夜
	UPROPERTY(EditAnywhere, Category = "Night") FVector MoonDirEnu = FVector(-0.40, 0.69, 0.60);
	UPROPERTY(EditAnywhere, Category = "Night") float MoonLux = 0.25f;           // 満月
	UPROPERTY(EditAnywhere, Category = "Night") FLinearColor MoonColor = FLinearColor(0.75f, 0.82f, 1.0f);
	UPROPERTY(EditAnywhere, Category = "Night") float NightSkyNits = 0.08f;      // パノラマの値 1 の輝度（月夜の空 約 0.03 cd/m²）
	UPROPERTY(EditAnywhere, Category = "Night") FLinearColor NightTint = FLinearColor(0.45f, 0.6f, 1.0f);
	UPROPERTY(EditAnywhere, Category = "Night") float MoonDiskNits = 2500.0f;    // 満月の輝度
	UPROPERTY(EditAnywhere, Category = "Night") float StarNits = 0.6f;
	UPROPERTY(EditAnywhere, Category = "Night") float NightEV100 = 0.5f;          // −1.5・0.5・1.5 を比べて決めた（−1.5 は夕方のように明るい）

	/** 起動の引数で決まる（BeginPlay の順に関わらず、ほかのアクタからいつでも読める） */
	static bool IsNight();
	/** いま空にある光源（昼は太陽、夜は月）の向き（ENU、単位ベクトル） */
	FVector LightDirEnu() const { return (IsNight() ? MoonDirEnu : SunDirEnu).GetSafeNormal(); }

private:
	void Apply();

	bool bNight = false;
	UPROPERTY() TObjectPtr<UMaterialInstanceDynamic> SkyMid;
};
