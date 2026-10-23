// HakoniwaShipEditor.cpp
// 海の場面を組む入口（SHP-24 Step 1）。hakoniwa-unreal-rover の HakoniwaRoverEditor と同じ形。
//   自動：UnrealEditor.exe <uproject> -ShipBuildScene [-ShipMap=/Game/...] [-ShipQuit]
//   手で：エディタのコンソールで  Ship.BuildScene [-ShipMap=/Game/...]
//
// Step 1 の場面は骨組みだけ：太陽（平行光）・空の大気・空の光・高さの霧・仮の海面（平らな板。Gerstner 波面は Step 3）・観客の開始位置。

#include "Components/DirectionalLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Containers/Ticker.h"
#include "Editor.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "FileHelpers.h"
#include "GameFramework/PlayerStart.h"
#include "GameFramework/WorldSettings.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"
#include "ShipFrame.h"
#include "ShipViewGameMode.h"

DEFINE_LOG_CATEGORY_STATIC(LogShipEditor, Log, All);

namespace
{
	const TCHAR* DefaultMap = TEXT("/Game/Ship/Levels/ShipSea");

	FString ParseMap(const TCHAR* Cmd)
	{
		FString Map = DefaultMap;
		FParse::Value(Cmd, TEXT("-ShipMap="), Map);
		return Map;
	}

	bool BuildScene()
	{
		UWorld* World = GEditor ? GEditor->NewMap(false) : nullptr;
		if (!World)
		{
			return false;
		}
		World->GetWorldSettings()->DefaultGameMode = AShipViewGameMode::StaticClass();

		// 太陽：仰角 35°・方位は南東から（光は太陽から来る向き）。
		// ★ ADirectionalLight は生成時に光へピッチ −46° を入れていて、スポーンの回転に足される（rover の README）。回転は生成のあとで決める
		const FVector ToSunUe = ShipFrame::LocalDirToUnreal(0.57, -0.57, 0.57);
		ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>();
		Sun->SetActorRotation((-ToSunUe).Rotation());
		Sun->SetActorLabel(TEXT("Sun"));
		UDirectionalLightComponent* SunC = CastChecked<UDirectionalLightComponent>(Sun->GetLightComponent());
		SunC->SetMobility(EComponentMobility::Movable);
		SunC->SetIntensity(10.0f);
		SunC->SetAtmosphereSunLight(true);
		SunC->SetCastShadows(true);

		AActor* Sky = World->SpawnActor<AActor>();
		USkyAtmosphereComponent* SkyC = NewObject<USkyAtmosphereComponent>(Sky, TEXT("SkyAtmosphere"));
		Sky->SetRootComponent(SkyC);
		Sky->AddInstanceComponent(SkyC);
		SkyC->RegisterComponent();
		Sky->SetActorLabel(TEXT("SkyAtmosphere"));

		ASkyLight* SkyLight = World->SpawnActor<ASkyLight>();
		SkyLight->SetActorLabel(TEXT("SkyLight"));
		USkyLightComponent* SkyLightC = SkyLight->GetLightComponent();
		SkyLightC->SetMobility(EComponentMobility::Movable);
		SkyLightC->bRealTimeCapture = true;

		AExponentialHeightFog* Fog = World->SpawnActor<AExponentialHeightFog>();
		Fog->SetActorLabel(TEXT("HeightFog"));

		// 仮の海面：平均海面（z = 0）に 4 km 四方の板（エンジンの Plane は 1 m 四方）
		AStaticMeshActor* Sea = World->SpawnActor<AStaticMeshActor>();
		Sea->SetActorLabel(TEXT("SeaPlaceholder"));
		UStaticMesh* Plane = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Plane.Plane"));
		if (!Plane)
		{
			UE_LOG(LogShipEditor, Error, TEXT("/Engine/BasicShapes/Plane not found"));
			return false;
		}
		Sea->GetStaticMeshComponent()->SetStaticMesh(Plane);
		Sea->SetActorLocation(ShipFrame::LocalToUnreal(0.0, 0.0, 0.0));
		Sea->SetActorScale3D(FVector(4000.0, 4000.0, 1.0));

		// 観客の開始位置：原点の南西 約 280 m・高さ 40 m から北東を見る
		APlayerStart* Start = World->SpawnActor<APlayerStart>(ShipFrame::LocalToUnreal(-200.0, -200.0, 40.0),
			ShipFrame::LocalDirToUnreal(1.0, 1.0, -0.15).Rotation());
		Start->SetActorLabel(TEXT("ViewStart"));

		UE_LOG(LogShipEditor, Log, TEXT("scene built: sun, sky, fog, sea placeholder, view start"));
		return true;
	}

	bool SaveScene(const FString& Map)
	{
		UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World)
		{
			return false;
		}
		const bool bMap = UEditorLoadingAndSavingUtils::SaveMap(World, Map);
		const bool bRest = UEditorLoadingAndSavingUtils::SaveDirtyPackages(false, true);
		UE_LOG(LogShipEditor, Log, TEXT("saved map %s (%d) and content (%d)"), *Map, bMap, bRest);
		return bMap && bRest;
	}

	/** 組む → 数十フレーム待つ（シェーダ）→ 保存 →（必要なら）終了。 */
	void RunScheduled(const FString& Map, bool bQuit, int32 StartDelayFrames)
	{
		struct FState { int32 Frame = 0; bool bBuilt = false; };
		TSharedRef<FState> St = MakeShared<FState>();
		FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Map, bQuit, StartDelayFrames, St](float) -> bool
		{
			++St->Frame;
			if (!GEditor || St->Frame < StartDelayFrames)
			{
				return true;
			}
			if (!St->bBuilt)
			{
				St->bBuilt = true;
				St->Frame = StartDelayFrames;
				if (!BuildScene())
				{
					UE_LOG(LogShipEditor, Error, TEXT("Ship.BuildScene failed"));
					if (bQuit) { FPlatformMisc::RequestExitWithStatus(false, 1); }
					return false;
				}
				return true;
			}
			if (St->Frame < StartDelayFrames + 30)
			{
				return true;
			}
			const bool bSaved = SaveScene(Map);
			UE_LOG(LogShipEditor, Log, TEXT("Ship.BuildScene %s"), bSaved ? TEXT("OK") : TEXT("save failed"));
			if (bQuit)
			{
				FPlatformMisc::RequestExitWithStatus(false, bSaved ? 0 : 1);
			}
			return false;
		}));
	}
}

class FHakoniwaShipEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		Command = MakeUnique<FAutoConsoleCommand>(
			TEXT("Ship.BuildScene"),
			TEXT("Build the sea scene: [-ShipMap=/Game/...]"),
			FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Argv)
			{
				RunScheduled(ParseMap(*(TEXT(" ") + FString::Join(Argv, TEXT(" ")))), false, 1);
			}));
		const TCHAR* Cmd = FCommandLine::Get();
		if (FParse::Param(Cmd, TEXT("ShipBuildScene")))
		{
			RunScheduled(ParseMap(Cmd), FParse::Param(Cmd, TEXT("ShipQuit")), 120);
		}
	}

	virtual void ShutdownModule() override
	{
		Command.Reset();
	}

private:
	TUniquePtr<FAutoConsoleCommand> Command;
};

IMPLEMENT_MODULE(FHakoniwaShipEditorModule, HakoniwaShipEditor);
