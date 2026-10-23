// ShipHud.cpp

#include "ShipHud.h"

#include "Camera/PlayerCameraManager.h"
#include "Components/InputComponent.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/TextureRenderTarget2D.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "ShipActor.h"
#include "ShipCamera.h"
#include "ShipFleet.h"
#include "ShipFrame.h"
#include "ShipOcean.h"

namespace
{
	const FLinearColor Panel(0.0f, 0.0f, 0.0f, 0.55f);
	const FLinearColor White(0.92f, 0.95f, 1.0f);
	const FLinearColor Dim(0.55f, 0.62f, 0.7f);
	const FLinearColor Cyan(0.3f, 0.9f, 1.0f);
	const FLinearColor Green(0.35f, 1.0f, 0.45f);
	const FLinearColor Yellow(1.0f, 0.85f, 0.2f);
	const FLinearColor Red(1.0f, 0.25f, 0.2f);
	const FLinearColor Orange(1.0f, 0.55f, 0.1f);
	constexpr float Line = 22.0f;   // 1 行の高さ [px]（中くらいの字体）

	FString Deg3(double D) { return FString::Printf(TEXT("%05.1f"), ShipNav::Wrap360(D)); }

	FLinearColor TargetColor(const ShipNav::FTarget& T)
	{
		return T.Cpa.collision_risk ? Red : (T.bAis ? Green : Yellow);
	}

	FString TargetName(const ShipNav::FTarget& T)
	{
		return T.bAis && !T.Name.IsEmpty() ? T.Name : TEXT("NON-AIS (radar)");
	}

	FString TcpaText(const ShipNav::FTarget& T)
	{
		return T.Cpa.tcpa_minutes >= 0.0 ? FString::Printf(TEXT("%.1f min"), T.Cpa.tcpa_minutes) : TEXT("passed");
	}
}

void AShipHud::BeginPlay()
{
	Super::BeginPlay();
	if (PlayerOwner)
	{
		EnableInput(PlayerOwner);
		if (InputComponent)
		{
			// 画面の切り替え（1/2/3/0）。観客の操作（WASD など）を妨げないよう入力は消費しない
			FInputKeyBinding& B = InputComponent->BindKey(EKeys::AnyKey, IE_Pressed, this, &AShipHud::OnAnyKey);
			B.bConsumeInput = false;
		}
	}
}

void AShipHud::OnAnyKey(FKey Key)
{
	AShipFleet* F = Fleet();
	if (!F) { return; }
	if (Key == EKeys::One) { F->SetView(EShipView::Chase); }
	else if (Key == EKeys::Two) { F->SetView(EShipView::Bridge); }
	else if (Key == EKeys::Three) { F->SetView(EShipView::Foc); }
	else if (Key == EKeys::Zero) { F->SetView(EShipView::Free); }
}

AShipFleet* AShipHud::Fleet()
{
	if (!FleetPtr.IsValid())
	{
		for (TActorIterator<AShipFleet> It(GetWorld()); It; ++It)
		{
			FleetPtr = *It;
			break;
		}
	}
	return FleetPtr.Get();
}

void AShipHud::Text(const FString& S, float X, float Y, const FLinearColor& C, float Scale, bool bLarge)
{
	DrawText(S, C, X, Y, bLarge ? GEngine->GetLargeFont() : GEngine->GetMediumFont(), Scale);
}

void AShipHud::Box(float X, float Y, float W, float H, const FLinearColor& C)
{
	DrawRect(C, X, Y, W, H);
}

void AShipHud::Circle(float Cx, float Cy, float R, const FLinearColor& C, float Thickness, int32 Seg)
{
	for (int32 i = 0; i < Seg; ++i)
	{
		const float A0 = 2.0f * PI * i / Seg, A1 = 2.0f * PI * (i + 1) / Seg;
		DrawLine(Cx + R * FMath::Cos(A0), Cy + R * FMath::Sin(A0), Cx + R * FMath::Cos(A1), Cy + R * FMath::Sin(A1), C, Thickness);
	}
}

void AShipHud::DrawHUD()
{
	Super::DrawHUD();
	AShipFleet* F = Fleet();
	if (!F || !Canvas)
	{
		return;
	}
	switch (F->GetView())
	{
	case EShipView::Bridge: DrawBridge(F); break;
	case EShipView::Foc: DrawFoc(F); break;
	default: break;
	}
}

void AShipHud::DrawNavPanel(AShipFleet* F, float X, float Y, float W)
{
	// 自船の航海情報・海況・自動化レベル（ブリッジ視界の左上、FOC の右下）
	if (F->ViewShips.Num() == 0) { return; }
	const AShipFleet::FShipView& Own = F->ViewShips[0];
	const ShipNav::FState& S = Own.State;
	constexpr int32 Rows = 6;
	Box(X, Y, W, Rows * Line + 12.0f, Panel);
	float Ty = Y + 6.0f;
	const FString Mode = F->IsSynced() ? FString::Printf(TEXT("T %.1f s"), F->ViewTime) : TEXT("OFFLINE PREVIEW");
	Text(FString::Printf(TEXT("OWN  %s (%s)    %s"), *Own.Name, *Own.Robot, *Mode), X + 10, Ty, Cyan); Ty += Line;
	Text(FString::Printf(TEXT("HDG %s   COG %s   SOG %5.2f kn"), *Deg3(S.HeadingDeg), *Deg3(S.CogDeg), S.SogKt), X + 10, Ty, White); Ty += Line;
	const FString Rudder = F->bHasJoints ? FString::Printf(TEXT("%s %4.1f"), F->RudderDeg >= 0.0 ? TEXT("S") : TEXT("P"), FMath::Abs(F->RudderDeg)) : TEXT("--");
	const FString Rpm = F->bHasJoints ? FString::Printf(TEXT("%.0f rpm"), F->PropRpm) : TEXT("--");
	Text(FString::Printf(TEXT("ROT %+5.1f /min   RUDDER %s   PROP %s"), S.RotDegMin, *Rudder, *Rpm), X + 10, Ty, White); Ty += Line;
	if (const AShipOcean* O = F->GetOcean())
	{
		const ShipWave::FParams& W = O->GetWaveParams();
		const FString Sea = O->IsWaveActive()
			? FString::Printf(TEXT("WAVE Hs %.2f m  Tp %.1f s  to %s   DEPTH %.0f m"), W.hs_m, W.tp_s, *Deg3(90.0 - FMath::RadiansToDegrees(W.wave_dir_rad)), W.water_depth_m)
			: FString(TEXT("WAVE calm"));
		Text(Sea, X + 10, Ty, White);
	}
	Ty += Line;
	int32 Risks = 0;
	for (const ShipNav::FTarget& T : F->Targets) { Risks += T.Cpa.collision_risk ? 1 : 0; }
	Text(FString::Printf(TEXT("TARGETS %d   COLLISION RISK %d"), F->Targets.Num(), Risks), X + 10, Ty, Risks > 0 ? Red : White); Ty += Line;
	// 自動化レベル（ch 50、D29）：AL・ODD 余裕度・航行状態・指令針路/速力。1 s 以上届かなければ古いと示す
	if (F->AutonomyFrames > 0 && F->AutonomyLevel > 0)
	{
		const bool bStale = F->ViewTime - F->AutonomyLastT > 1.0;
		const FLinearColor C = bStale ? Dim : (F->AutonomyLevel >= 4 ? Green : (F->AutonomyLevel == 3 ? Yellow : Red));
		FString Cmd = F->AutonomyCourseDeg >= 0.0 ? FString::Printf(TEXT("  CMD %s %.1f kn"), *Deg3(F->AutonomyCourseDeg), F->AutonomySpeedKt) : FString();
		Text(FString::Printf(TEXT("AUTONOMY AL%d  ODD %.2f  %s%s%s%s"), F->AutonomyLevel, F->AutonomyOddMargin, *F->AutonomyNav, *Cmd,
			F->bAutonomyMrs ? TEXT("  MRS") : TEXT(""), bStale ? TEXT("  (stale)") : TEXT("")), X + 10, Ty, C);
	}
	else
	{
		const FString Auto = F->AutonomyFrames > 0
			? FString::Printf(TEXT("AUTONOMY  unreadable ch 50 (%lld)"), F->AutonomyFrames)
			: FString(TEXT("AUTONOMY  no data (ch 50 autonomy_status)"));
		Text(Auto, X + 10, Ty, F->AutonomyFrames > 0 ? Yellow : Dim);
	}
}

void AShipHud::DrawHeadingTape(AShipFleet* F)
{
	// 画面上の船首方位の目盛り（カメラの向き）。自船の HDG（白）と COG（水色）の印
	if (!PlayerOwner || !PlayerOwner->PlayerCameraManager || F->ViewShips.Num() == 0) { return; }
	const float ViewHdg = 90.0f + PlayerOwner->PlayerCameraManager->GetCameraRotation().Yaw;   // Unreal の yaw 0 = 東、+90 = 南
	const float Cx = Canvas->ClipX * 0.5f, W = 640.0f, Y = 14.0f, Span = 40.0f;
	Box(Cx - W * 0.5f - 6, Y - 4, W + 12, 44, Panel);
	for (int32 d = FMath::FloorToInt(ViewHdg - Span); d <= FMath::CeilToInt(ViewHdg + Span); ++d)
	{
		if (d % 5 != 0) { continue; }
		const float X = Cx + (d - ViewHdg) / Span * W * 0.5f;
		const bool bMajor = d % 10 == 0;
		DrawLine(X, Y, X, Y + (bMajor ? 12 : 6), White, 1.5f);
		if (bMajor)
		{
			const int32 C = ((d % 360) + 360) % 360;
			const FString L = C == 0 ? TEXT("N") : C == 90 ? TEXT("E") : C == 180 ? TEXT("S") : C == 270 ? TEXT("W") : FString::Printf(TEXT("%03d"), C);
			Text(L, X - 12, Y + 14, White, 0.8f);
		}
	}
	const ShipNav::FState& S = F->ViewShips[0].State;
	auto Mark = [&](double Hdg, const FLinearColor& C)
	{
		const float D = ShipNav::Wrap180(Hdg - ViewHdg);
		if (FMath::Abs(D) <= Span)
		{
			const float X = Cx + D / Span * W * 0.5f;
			DrawLine(X - 6, Y - 2, X + 6, Y - 2, C, 3.0f);
			DrawLine(X, Y - 2, X, Y + 14, C, 2.0f);
		}
	};
	Mark(S.HeadingDeg, White);
	Mark(S.CogDeg, Cyan);
}

void AShipHud::DrawTargetTags(AShipFleet* F)
{
	// 画面上の他船の位置に、船名・距離・方位・CPA/TCPA のタグ（AR）
	if (!PlayerOwner || !PlayerOwner->PlayerCameraManager) { return; }
	const FVector CamPos = PlayerOwner->PlayerCameraManager->GetCameraLocation();
	const FVector CamFwd = PlayerOwner->PlayerCameraManager->GetCameraRotation().Vector();
	for (const ShipNav::FTarget& T : F->Targets)
	{
		const TObjectPtr<AShipActor>* Ship = F->Ships.FindByPredicate([&T](const TObjectPtr<AShipActor>& A) { return A && A->Robot == T.Robot; });
		if (!Ship) { continue; }
		const FBox B = (*Ship)->GetComponentsBoundingBox(true);
		const FVector Anchor(B.GetCenter().X, B.GetCenter().Y, B.Max.Z);
		if (FVector::DotProduct(Anchor - CamPos, CamFwd) <= 0.0) { continue; }   // カメラの後ろ
		FVector2D P;
		if (!PlayerOwner->ProjectWorldLocationToScreen(Anchor, P, false)) { continue; }
		if (P.X < 0 || P.Y < 0 || P.X > Canvas->ClipX || P.Y > Canvas->ClipY) { continue; }
		const FLinearColor C = TargetColor(T);
		const float Tw = 250.0f, Th = 3 * Line + 8.0f, Tx = FMath::Clamp(P.X - Tw * 0.5f, 4.0f, Canvas->ClipX - Tw - 4.0f), Ty = P.Y - 30.0f - Th;
		DrawLine(P.X, P.Y, P.X, Ty + Th, C, 1.5f);
		Box(Tx, Ty, Tw, Th, Panel);
		DrawLine(Tx, Ty, Tx + Tw, Ty, C, 2.0f);
		Text(TargetName(T), Tx + 6, Ty + 4, C);
		Text(FString::Printf(TEXT("%.2f NM  BRG %s (R%+.0f)"), T.RangeNm, *Deg3(T.BearingDeg), T.RelBearingDeg), Tx + 6, Ty + 4 + Line, White, 0.85f);
		Text(FString::Printf(TEXT("CPA %.2f NM  TCPA %s"), T.Cpa.cpa_distance_nm, *TcpaText(T)), Tx + 6, Ty + 4 + 2 * Line, C, 0.85f);
	}
}

void AShipHud::DrawBridge(AShipFleet* F)
{
	DrawHeadingTape(F);
	DrawTargetTags(F);
	DrawNavPanel(F, 16.0f, 70.0f, 560.0f);
	Text(TEXT("BRIDGE VIEW   [1] chase  [2] bridge  [3] FOC  [0] free"), 16.0f, Canvas->ClipY - 30.0f, Dim, 0.8f);
}

void AShipHud::DrawChart(AShipFleet* F, float X0, float Y0, float Sz)
{
	// 北が上・自船中心の海図。距離環（0.5 NM）、各船の向き・速度ベクトル（AIS ベクトルと同じ 6 分）・航跡、OZT の危険進路の弧
	Box(X0, Y0, Sz, Sz, FLinearColor(0.02f, 0.06f, 0.12f, 1.0f));
	if (F->ViewShips.Num() == 0 || !F->ViewShips[0].State.bValid) { return; }
	const ShipNav::FState& Own = F->ViewShips[0].State;
	const float Cx = X0 + Sz * 0.5f, Cy = Y0 + Sz * 0.5f;
	const double RangeM = F->DisplayRangeM, Scale = (Sz * 0.5) / RangeM;
	auto ToScreen = [&](double E, double N) { return FVector2D(Cx + (E - Own.E) * Scale, Cy - (N - Own.N) * Scale); };
	auto Inside = [&](const FVector2D& P) { return P.X >= X0 && P.X <= X0 + Sz && P.Y >= Y0 && P.Y <= Y0 + Sz; };
	for (double R = 926.0; R <= RangeM + 1.0; R += 926.0)
	{
		Circle(Cx, Cy, R * Scale, FLinearColor(0.2f, 0.35f, 0.5f), 1.0f);
		Text(FString::Printf(TEXT("%.1f"), R / 1852.0), Cx + 4, Cy - R * Scale - 18, Dim, 0.7f);
	}
	Text(TEXT("N"), Cx - 6, Y0 + 4, White);
	// OZT：自船が取ると他船と安全離隔（926 m）以内に近づく進路の弧（Godot の扇形と同じ値）
	const float Rarc = Sz * 0.5f * 0.92f;
	for (const ShipNav::FTarget& T : F->Targets)
	{
		for (const auto& A : T.Ozt.danger_arcs)
		{
			double Span = A.end_deg - A.start_deg;
			if (Span < 0.0) { Span += 360.0; }
			const int32 Steps = FMath::Max(2, FMath::CeilToInt(Span / 2.0));
			for (int32 i = 0; i < Steps; ++i)
			{
				const double B0 = FMath::DegreesToRadians(A.start_deg + Span * i / Steps), B1 = FMath::DegreesToRadians(A.start_deg + Span * (i + 1) / Steps);
				DrawLine(Cx + Rarc * FMath::Sin(B0), Cy - Rarc * FMath::Cos(B0), Cx + Rarc * FMath::Sin(B1), Cy - Rarc * FMath::Cos(B1), Orange, 5.0f);
			}
		}
	}
	for (int32 i = 0; i < F->ViewShips.Num(); ++i)
	{
		const AShipFleet::FShipView& V = F->ViewShips[i];
		if (!V.State.bValid) { continue; }
		const ShipNav::FTarget* T = i > 0 && i - 1 < F->Targets.Num() ? &F->Targets[i - 1] : nullptr;
		const FLinearColor C = i == 0 ? Cyan : TargetColor(*T);
		for (const FVector2D& Tp : V.Trail)
		{
			const FVector2D P = ToScreen(Tp.X, Tp.Y);
			if (Inside(P)) { DrawRect(C * 0.6f, P.X - 1, P.Y - 1, 2, 2); }
		}
		const FVector2D P = ToScreen(V.State.E, V.State.N);
		if (!Inside(P)) { continue; }
		const double Vt = F->VectorMinutes * 60.0;
		const FVector2D Pv = ToScreen(V.State.E + V.State.VelE * Vt, V.State.N + V.State.VelN * Vt);
		DrawLine(P.X, P.Y, Pv.X, Pv.Y, C, 2.0f);
		// 船の向きの三角形（長さは実寸か 14 px の大きい方）
		const double H = FMath::DegreesToRadians(V.State.HeadingDeg);
		const float L = FMath::Max(14.0f, static_cast<float>(V.LengthM * Scale)), Wd = L * 0.45f;
		const FVector2D Fw(FMath::Sin(H), -FMath::Cos(H)), Rt(FMath::Cos(H), FMath::Sin(H));
		const FVector2D A = P + Fw * (L * 0.6f), Bp = P - Fw * (L * 0.4f) + Rt * (Wd * 0.5f), Cp = P - Fw * (L * 0.4f) - Rt * (Wd * 0.5f);
		DrawLine(A.X, A.Y, Bp.X, Bp.Y, C, 2.0f);
		DrawLine(Bp.X, Bp.Y, Cp.X, Cp.Y, C, 2.0f);
		DrawLine(Cp.X, Cp.Y, A.X, A.Y, C, 2.0f);
		const FString Label = i == 0 ? V.Name : (T ? TargetName(*T) : V.Robot);
		Text(Label, P.X + 10, P.Y + 6, C, 0.75f);
	}
	Text(FString::Printf(TEXT("RANGE %.1f NM  rings 0.5 NM  vectors %.0f min  OZT arcs (orange)"), RangeM / 1852.0, F->VectorMinutes), X0 + 8, Y0 + Sz - 24, Dim, 0.75f);
}

void AShipHud::DrawTargetTable(AShipFleet* F, float X, float Y, float W)
{
	const int32 Rows = F->Targets.Num() + 1;
	Box(X, Y, W, Rows * Line + 12.0f, Panel);
	float Ty = Y + 6.0f;
	const float Cols[] = { 10.0f, 0.34f * W, 0.5f * W, 0.62f * W, 0.76f * W, 0.89f * W };   // 表示板の幅に合わせる
	const TCHAR* Heads[] = { TEXT("TARGET"), TEXT("RANGE NM"), TEXT("BRG"), TEXT("CPA NM"), TEXT("TCPA"), TEXT("RISK") };
	for (int32 c = 0; c < 6; ++c) { Text(Heads[c], X + Cols[c], Ty, Dim, 0.85f); }
	Ty += Line;
	for (const ShipNav::FTarget& T : F->Targets)
	{
		const FLinearColor C = TargetColor(T);
		Text(TargetName(T), X + Cols[0], Ty, C, 0.85f);
		Text(FString::Printf(TEXT("%.2f"), T.RangeNm), X + Cols[1], Ty, White, 0.85f);
		Text(Deg3(T.BearingDeg), X + Cols[2], Ty, White, 0.85f);
		Text(FString::Printf(TEXT("%.2f"), T.Cpa.cpa_distance_nm), X + Cols[3], Ty, White, 0.85f);
		Text(TcpaText(T), X + Cols[4], Ty, White, 0.85f);
		Text(T.Cpa.collision_risk ? TEXT("ALARM") : TEXT("-"), X + Cols[5], Ty, C, 0.85f);
		Ty += Line;
	}
}

void AShipHud::DrawFoc(AShipFleet* F)
{
	// FOC：全面を覆う表示板（左に海図、右上にカメラ、右下に表）
	const float Cw = Canvas->ClipX, Ch = Canvas->ClipY;
	Box(0, 0, Cw, Ch, FLinearColor(0.01f, 0.02f, 0.04f, 1.0f));
	const float Sz = FMath::Min(Ch - 70.0f, Cw * 0.5f);
	Text(TEXT("FOC  SHORE CONTROL CENTRE"), 16.0f, 10.0f, Cyan, 1.0f, true);
	DrawChart(F, 16.0f, 54.0f, Sz);
	const float X1 = 16.0f + Sz + 20.0f, W = Cw - X1 - 16.0f;
	const TArray<TObjectPtr<UShipCamera>>& Cams = F->GetCameras();
	const float Tw = (W - 10.0f) / 2.0f, Th = Tw * 0.75f;
	for (int32 i = 0; i < Cams.Num() && i < 2; ++i)
	{
		const UShipCamera* Cam = Cams[i];
		const float Tx = X1 + i * (Tw + 10.0f), Ty = 54.0f + Line;
		Text(FString::Printf(TEXT("%s  (%s)  t %.1f s"), *Cam->FrameId, *Cam->PduName, Cam->LastStamp), Tx, 54.0f, White, 0.8f);
		if (Cam->GetTarget() && Cam->Captured > 0)
		{
			DrawTexture(Cam->GetTarget(), Tx, Ty, Tw, Th, 0, 0, 1, 1, FLinearColor::White, BLEND_Opaque);
		}
		else
		{
			Box(Tx, Ty, Tw, Th, Panel);
			Text(TEXT("no image"), Tx + 10, Ty + 10, Dim);
		}
	}
	const float Yt = 54.0f + Line + Th + 16.0f;
	DrawNavPanel(F, X1, Yt, W);
	DrawTargetTable(F, X1, Yt + 6 * Line + 24.0f, W);
	Text(TEXT("[1] chase  [2] bridge  [3] FOC  [0] free"), X1, Ch - 30.0f, Dim, 0.8f);
}
