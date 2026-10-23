// ShipWave.h
// 物理と同じ決定論的波面（D4、SHP-24 Step 3）。hakoniwa-mujoco-ship の wave_spectrum.hpp をそのまま使う
// （tools/gen_fleet_config.py が Public/ThirdParty/wave_spectrum.hpp に写す。内容は変えない）。
// 座標は ENU（x 東・y 北・z 上）[m]、時刻は物理時刻 [s]（D19）。

#pragma once

#include "CoreMinimal.h"

#include <algorithm>

THIRD_PARTY_INCLUDES_START
// ★ UE は PI をマクロで定義している（UnrealMathUtility.h）。wave_spectrum.hpp の constexpr PI と衝突するので一時的に外す
#pragma push_macro("PI")
#undef PI
#include "ThirdParty/wave_spectrum.hpp"
#pragma pop_macro("PI")
THIRD_PARTY_INCLUDES_END

namespace ShipWave
{
	using FParams = ship::environment::WaveParams;
	using FEngine = ship::environment::WaveSpectrumEngine;

	/** 物理プラグイン（ship_hydro_plugin.cpp の update_wave_engine_if_needed）と同じ成分数・有効化の閾値（fleet_ue.json の wave で上書き） */
	constexpr int32 DefaultComponents = 32;
	constexpr double DefaultActiveHs = 0.01;

	/**
	 * Disturbance PDU の d_user_custom から波浪パラメータを取り出す（ShipHydroPlugin::parse_from_custom と同じ配置、Godot の disturbance_sync.gd と同じ）。
	 *   [1].data[4] = water_depth_m、[2].data = [hs_m, tp_s, wave_dir_rad, gamma, seed]
	 * 物理プラグインは潮位（[1].data[5]）を波面に入れていないので tide_level_m = 0。波浪成分が無ければ false。
	 */
	inline bool ParseDisturbance(const TArray<TArray<double>>& Custom, FParams& Out)
	{
		if (Custom.Num() < 3 || Custom[1].Num() < 5 || Custom[2].Num() < 5)
		{
			return false;
		}
		Out = FParams();
		Out.hs_m = Custom[2][0];
		Out.tp_s = Custom[2][1];
		Out.wave_dir_rad = Custom[2][2];
		Out.gamma = Custom[2][3];
		Out.seed = static_cast<uint32_t>(Custom[2][4]);
		Out.water_depth_m = Custom[1][4];
		Out.tide_level_m = 0.0;
		return true;
	}

	inline bool SameParams(const FParams& A, const FParams& B)
	{
		return A.hs_m == B.hs_m && A.tp_s == B.tp_s && A.wave_dir_rad == B.wave_dir_rad && A.gamma == B.gamma
			&& A.seed == B.seed && A.water_depth_m == B.water_depth_m && A.tide_level_m == B.tide_level_m
			&& A.num_components == B.num_components;
	}
}
