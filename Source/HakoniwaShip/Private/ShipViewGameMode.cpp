// ShipViewGameMode.cpp

#include "ShipViewGameMode.h"

#include "GameFramework/PlayerController.h"
#include "GameFramework/SpectatorPawn.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "EngineUtils.h"
#include "Engine/PostProcessVolume.h"
#include "Misc/Paths.h"
#include "UnrealClient.h"

DEFINE_LOG_CATEGORY_STATIC(LogShipView, Log, All);

AShipViewGameMode::AShipViewGameMode()
{
	DefaultPawnClass = ASpectatorPawn::StaticClass();
	PrimaryActorTick.bCanEverTick = true;
}

void AShipViewGameMode::PostLogin(APlayerController* NewPlayer)
{
	Super::PostLogin(NewPlayer);
	if (NewPlayer)
	{
		// Config/DefaultInput.ini の NoCapture・DoNotLock と合わせて、カーソルを出したままにする
		NewPlayer->bShowMouseCursor = true;
		FInputModeGameAndUI Mode;
		Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
		Mode.SetHideCursorDuringCapture(false);
		NewPlayer->SetInputMode(Mode);
	}
}

void AShipViewGameMode::StartPlay()
{
	Super::StartPlay();
	float Ev = 0.0f;
	if (FParse::Value(FCommandLine::Get(), TEXT("-ShipExposure="), Ev))
	{
		// 露出の調整用：場面の DaylightExposure（無限の範囲のポストプロセス）の EV100 を上書きする
		for (TActorIterator<APostProcessVolume> It(GetWorld()); It; ++It)
		{
			It->Settings.bOverride_AutoExposureMinBrightness = true;
			It->Settings.bOverride_AutoExposureMaxBrightness = true;
			It->Settings.AutoExposureMinBrightness = Ev;
			It->Settings.AutoExposureMaxBrightness = Ev;
		}
		UE_LOG(LogShipView, Log, TEXT("exposure EV100 = %.2f"), Ev);
	}
	float Sec = 0.0f;
	if (FParse::Value(FCommandLine::Get(), TEXT("-ShipSmoke="), Sec) && Sec > 0.0f)
	{
		SmokeSec = Sec;
		UE_LOG(LogShipView, Log, TEXT("smoke: drawing for %.1f s"), SmokeSec);
	}
}

void AShipViewGameMode::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (ExitCountdown >= 0 && --ExitCountdown < 0)
	{
		FPlatformMisc::RequestExitWithStatus(false, 0);
	}
	if (SmokeSec <= 0.0 || bSmokeDone)
	{
		return;
	}
	const double Now = FPlatformTime::Seconds();
	if (SmokeStart < 0.0)
	{
		// 最初のフレーム（シェーダの読み込みなど）は数えない
		SmokeStart = Now;
		return;
	}
	++SmokeFrames;
	if (Now - SmokeStart >= SmokeSec)
	{
		bSmokeDone = true;
		const double Fps = SmokeFrames / (Now - SmokeStart);
		UE_LOG(LogShipView, Display, TEXT("HAKONIWA_UNREAL_SHIP_BOOT_OK frames=%lld fps=%.1f map=%s"),
			SmokeFrames, Fps, *GetWorld()->GetMapName());
		FString Shot;
		if (FParse::Value(FCommandLine::Get(), TEXT("-ShipShot="), Shot) && !FParse::Param(FCommandLine::Get(), TEXT("ShipSync")))
		{
			// 撮影は次の描画で行われるので、数フレーム待ってから終わる
			FScreenshotRequest::RequestScreenshot(FPaths::ConvertRelativePathToFull(Shot), true, false);   // 画面の警告（赤い文字）も写す
			ExitCountdown = 30;
			return;
		}
		FPlatformMisc::RequestExitWithStatus(false, 0);
	}
}
