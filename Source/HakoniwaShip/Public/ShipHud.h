// ShipHud.h
// ブリッジ視界と FOC（陸上監視センター）画面（SHP-25 Step 2）。値は AShipFleet が表示時刻で求めたもの（ShipNav、Godot の工学オーバレイと同じ求め方）。
//
//   キー 1：自船を追う  2：船橋からの視界  3：FOC  0：自由（観客）   （起動の引数 -ShipView=chase|bridge|foc|free）
//   船橋：自船の航海情報（HDG・COG・SOG・ROT・舵角・プロペラ回転数・海況・時刻）の表示板と、画面上の他船の位置に
//         船名・距離・方位・CPA/TCPA のタグ（衝突の危険は赤、AIS の無い船は黄で「NON-AIS」）。上に船首方位の目盛り
//   FOC：左に北が上の海図（自船中心、距離環、各船の向き・6 分の速度ベクトル・航跡、OZT の危険進路の弧）、右上に船首・船橋カメラの画像、
//        右下に自船・海況・他船の表と、自動化レベル（autonomy_status ch 50。形式が未定なので受け取ったかだけ、K11）
// 表記は英語（船橋の計器と同じ略号。エンジンの既定の字体に日本語の字形が無い）。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "ShipHud.generated.h"

class AShipFleet;

UCLASS()
class HAKONIWASHIP_API AShipHud : public AHUD
{
	GENERATED_BODY()

public:
	virtual void BeginPlay() override;
	virtual void DrawHUD() override;

private:
	AShipFleet* Fleet();
	void DrawBridge(AShipFleet* F);
	void DrawFoc(AShipFleet* F);
	void DrawNavPanel(AShipFleet* F, float X, float Y, float W);
	void DrawTargetTags(AShipFleet* F);
	void DrawHeadingTape(AShipFleet* F);
	void DrawChart(AShipFleet* F, float X, float Y, float Size);
	void DrawTargetTable(AShipFleet* F, float X, float Y, float W);
	void Text(const FString& S, float X, float Y, const FLinearColor& C, float Scale = 1.0f, bool bLarge = false);
	void Box(float X, float Y, float W, float H, const FLinearColor& C);
	void Circle(float Cx, float Cy, float R, const FLinearColor& C, float Thickness = 1.0f, int32 Seg = 72);

	void OnAnyKey(FKey Key);

	TWeakObjectPtr<AShipFleet> FleetPtr;
};
