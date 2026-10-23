// ShipNav.h
// ブリッジ視界・FOC 画面の航海情報（SHP-25 Step 2）。Godot ビューアの工学オーバレイ（ship_overlays.gd・cpa_ozt.gd）と同じ求め方：
//   - 対地速度：位置の時系列（物理時刻つきの受信値）から、窓（fleet_ue.json の overlays.velocity_window_s、5 s）の最小二乗で等速直線の速度
//     （物理側に COG/SOG の PDU が無いため。ADR D16）。SOG [kt]・COG [deg]（真方位、北から右回り）
//   - 船首方位 HDG [deg] = 90° − yaw（yaw は東から左回り、D14）。回頭角速度 ROT [deg/min]（右回りが正）は同じ窓の船首方位の最小二乗
//   - CPA/TCPA/BCR・OZT：物理の cpa_ozt.hpp（CpaOztCalculator）をそのまま使う（gen_fleet_config.py が写し、--check で同じか確かめる）
// 座標は ENU（東, 北）[m]。

#pragma once

#include "CoreMinimal.h"

#include <algorithm>

THIRD_PARTY_INCLUDES_START
#include "ThirdParty/cpa_ozt.hpp"
THIRD_PARTY_INCLUDES_END

namespace ShipNav
{
	using FCpa = hako::ship::autonomy::CpaMetrics;
	using FOzt = hako::ship::autonomy::OztSector;
	constexpr double MpsToKt = 1.0 / 0.514444;
	constexpr double MToNm = 1.0 / 1852.0;

	/** 物理時刻つきの位置・向きの標本 */
	struct FSample
	{
		double T = 0.0;
		double E = 0.0;
		double N = 0.0;
		double Yaw = 0.0;   // [rad]、東から左回り
	};

	/** 1 隻の航海情報（時刻 T の値） */
	struct FState
	{
		bool bValid = false;
		double T = 0.0;
		double E = 0.0, N = 0.0;
		double HeadingDeg = 0.0;   // 船首方位（真方位）
		double VelE = 0.0, VelN = 0.0;
		double SogKt = 0.0;
		double CogDeg = 0.0;
		double RotDegMin = 0.0;    // 回頭角速度（右回りが正）
	};

	/** 他船（自船から見た）の諸元 */
	struct FTarget
	{
		FString Robot;
		FString Name;              // AIS の船名（AIS が無ければ空）
		bool bAis = false;
		double RangeNm = 0.0;
		double BearingDeg = 0.0;   // 真方位
		double RelBearingDeg = 0.0;// 自船首から右回り（−180〜180）
		FCpa Cpa;
		FOzt Ozt;
	};

	inline double Wrap360(double D) { D = FMath::Fmod(D, 360.0); return D < 0.0 ? D + 360.0 : D; }
	inline double Wrap180(double D) { D = Wrap360(D + 180.0); return D - 180.0; }
	inline double YawToHeadingDeg(double Yaw) { return Wrap360(90.0 - FMath::RadiansToDegrees(Yaw)); }

	/**
	 * 標本列（時刻の昇順）のうち T − Window ≤ t ≤ T のものから時刻 T の航海情報を求める（Godot の _fit_velocity と同じ最小二乗）。
	 * 位置・向きは T 以前の最後の標本の値（表示は補間した値だが、比べる物理の trace と同じ標本で求める）
	 */
	inline FState Estimate(const TArray<FSample>& H, double T, double Window)
	{
		FState S;
		int32 Last = INDEX_NONE;
		for (int32 i = H.Num() - 1; i >= 0; --i)
		{
			if (H[i].T <= T + 1e-9) { Last = i; break; }
		}
		if (Last == INDEX_NONE)
		{
			return S;
		}
		int32 First = Last;
		while (First > 0 && T - H[First - 1].T <= Window + 1e-9)
		{
			--First;
		}
		const int32 Count = Last - First + 1;
		S.bValid = true;
		S.T = H[Last].T;
		S.E = H[Last].E;
		S.N = H[Last].N;
		S.HeadingDeg = YawToHeadingDeg(H[Last].Yaw);
		if (Count < 2)
		{
			return S;
		}
		// 向きは折り返しを外してから最小二乗（右回り = 船首方位の増加 = yaw の減少）
		double Mt = 0.0, Me = 0.0, Mn = 0.0, My = 0.0;
		TArray<double> Unwrapped;
		Unwrapped.SetNum(Count);
		for (int32 i = 0; i < Count; ++i)
		{
			const FSample& X = H[First + i];
			Unwrapped[i] = i == 0 ? X.Yaw : Unwrapped[i - 1] + FMath::Atan2(FMath::Sin(X.Yaw - H[First + i - 1].Yaw), FMath::Cos(X.Yaw - H[First + i - 1].Yaw));
			Mt += X.T; Me += X.E; Mn += X.N; My += Unwrapped[i];
		}
		Mt /= Count; Me /= Count; Mn /= Count; My /= Count;
		double Stt = 0.0, Ste = 0.0, Stn = 0.0, Sty = 0.0;
		for (int32 i = 0; i < Count; ++i)
		{
			const FSample& X = H[First + i];
			const double Dt = X.T - Mt;
			Stt += Dt * Dt;
			Ste += Dt * (X.E - Me);
			Stn += Dt * (X.N - Mn);
			Sty += Dt * (Unwrapped[i] - My);
		}
		if (Stt <= 0.0)
		{
			return S;
		}
		S.VelE = Ste / Stt;
		S.VelN = Stn / Stt;
		S.SogKt = FMath::Sqrt(S.VelE * S.VelE + S.VelN * S.VelN) * MpsToKt;
		S.CogDeg = Wrap360(FMath::RadiansToDegrees(FMath::Atan2(S.VelE, S.VelN)));
		S.RotDegMin = -FMath::RadiansToDegrees(Sty / Stt) * 60.0;
		return S;
	}

	/** 自船から見た他船の諸元（cpa_ozt.hpp の CPA・OZT） */
	inline FTarget Relative(const FState& Own, const FState& Tgt, double CpaAlarmM, double TcpaAlarmS, double SafeDomainM, double HorizonS)
	{
		using hako::ship::autonomy::CpaOztCalculator;
		using hako::ship::autonomy::Vector2D;
		FTarget R;
		const double De = Tgt.E - Own.E, Dn = Tgt.N - Own.N;
		R.RangeNm = FMath::Sqrt(De * De + Dn * Dn) * MToNm;
		R.BearingDeg = Wrap360(FMath::RadiansToDegrees(FMath::Atan2(De, Dn)));
		R.RelBearingDeg = Wrap180(R.BearingDeg - Own.HeadingDeg);
		const Vector2D OwnPos { Own.E, Own.N }, OwnVel { Own.VelE, Own.VelN }, TgtPos { Tgt.E, Tgt.N }, TgtVel { Tgt.VelE, Tgt.VelN };
		R.Cpa = CpaOztCalculator::compute_cpa(OwnPos, OwnVel, FMath::DegreesToRadians(Own.HeadingDeg), TgtPos, TgtVel, CpaAlarmM, TcpaAlarmS);
		R.Ozt = CpaOztCalculator::compute_ozt(OwnPos, FMath::Sqrt(Own.VelE * Own.VelE + Own.VelN * Own.VelN), TgtPos, TgtVel, SafeDomainM, HorizonS);
		return R;
	}
}
