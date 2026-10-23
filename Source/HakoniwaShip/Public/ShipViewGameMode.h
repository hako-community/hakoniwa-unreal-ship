// ShipViewGameMode.h
// 海の場面を見るだけのゲームモード（hakoniwa-unreal-rover の RoverViewGameMode と同じ）。プレイヤーは姿の無い観客で、マウスは捕まえない。
//
// コマンドライン（-game のとき）:
//   -ShipSmoke=<秒>   起動の確認。指定秒だけ描いたあと、フレーム数と fps を
//                     「HAKONIWA_UNREAL_SHIP_BOOT_OK frames=<n> fps=<f>」としてログに書いて終わる（scripts\windows\ship_scene.ps1 -Smoke）
//   -ShipShot=<png>   -ShipSmoke の終わりに画面を撮ってから終わる（箱庭につながないとき。つなぐときは AShipFleet が撮る）

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ShipViewGameMode.generated.h"

UCLASS()
class HAKONIWASHIP_API AShipViewGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AShipViewGameMode();

	virtual void PostLogin(APlayerController* NewPlayer) override;
	virtual void StartPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	double SmokeSec = -1.0;
	double SmokeStart = -1.0;
	int64 SmokeFrames = 0;
	bool bSmokeDone = false;
	int32 ExitCountdown = -1;
};
