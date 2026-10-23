// ShipOceanShader.h
// 海面の材質の HLSL（SHP-24 Step 3）。エディタの場面づくり（HakoniwaShipEditor）が Custom 式に入れて材質を作る。
// 式は wave_spectrum.hpp の get_surface() と同じ（Godot の Shaders/wave_common.gdshaderinc と同じ形）。
//
// 波の表（AShipOcean が毎フレーム書く浮動小数のテクスチャ WaveTex、幅 ShipOceanShader::TexWidth・高さ 2）:
//   列 i < WaveCount（物理の成分）            行 0 = (kx, ky, amp, Q·amp/k)、行 1 = (位相, 0, 0, 0)
//   列 MaxWaves + j（j < RippleCount、見た目だけの細かい波） 行 0 = (kx, ky, amp, 0)、行 1 = (位相, 0, 0, 0)
//   位相 = phase − ω·t + kx·cE + ky·cN（mod 2π）を CPU が倍精度で畳み込む（cE, cN はメッシュの中心の ENU、t は物理時刻）。
//   シェーダは中心からの相対座標だけを単精度で扱う（Godot の shader_phases と同じ）。
// Rel はメッシュの中心からの Unreal の相対位置 [cm]（WorldPosition（材質の変位を除く）− ObjectPosition）。
// ENU へは 東 = X/100、北 = −Y/100。ENU の法線 (−sE, −sN, 1) は Unreal で (−sE, +sN, 1)。

#pragma once

#include "CoreMinimal.h"

namespace ShipOceanShader
{
	constexpr int32 MaxWaves = 64;
	constexpr int32 MaxRipples = 16;
	constexpr int32 TexWidth = MaxWaves + MaxRipples;

	/** 頂点：世界の変位 [cm]（WorldPositionOffset）。物理の成分だけ。HorizDisp = 0 なら上下だけで、物理の η(x, y, t) と同じ点で同じ高さ */
	inline const TCHAR* WpoCode()
	{
		return TEXT(R"(
float2 d = float2(Rel.x, -Rel.y) * 0.01;
float eta = 0.0;
float2 disp = float2(0.0, 0.0);
int n = (int)WaveCount;
for (int i = 0; i < n; i++)
{
	float4 c = WaveTex.Load(int3(i, 0, 0));
	float a = c.x * d.x + c.y * d.y + WaveTex.Load(int3(i, 1, 0)).x;
	float sa = sin(a);
	eta += c.z * cos(a);
	disp -= c.w * c.xy * sa;
}
float fade = 1.0 - smoothstep(FadeStart, FadeEnd, length(d));
eta *= fade;
disp *= fade * HorizDisp;
return float3(disp.x, -disp.y, eta + Tide) * 100.0;
)");
	}

	/** 画素：xyz = Unreal の世界の法線、w = 平均海面からの高さ [m]（外周で 0 へ落とす。頂点と同じ）。
	 *  傾きは外周でも落とさない（外側の平面も同じ式で傾きを出し、継ぎ目で明るさが変わらないように）。
	 *  遠くは傾きを弱める（0.5〜4 km で 15 % まで。画素より細かい波の法線はちらつき、水平線近くでは下向きに反射してしまう。そのばらつきは粗さで出す）。細かい波（見た目だけ）は傾きだけに足し、
	 *  カメラから 150〜900 m で消す（Depth = PixelDepth [cm]。遠くでちらつかないように） */
	inline const TCHAR* PixelCode()
	{
		return TEXT(R"(
float2 d = float2(Rel.x, -Rel.y) * 0.01;
float eta = 0.0;
float2 slope = float2(0.0, 0.0);
int n = (int)WaveCount;
for (int i = 0; i < n; i++)
{
	float4 c = WaveTex.Load(int3(i, 0, 0));
	float a = c.x * d.x + c.y * d.y + WaveTex.Load(int3(i, 1, 0)).x;
	eta += c.z * cos(a);
	slope -= c.z * c.xy * sin(a);
}
float fade = 1.0 - smoothstep(FadeStart, FadeEnd, length(d));
eta *= fade;
slope *= 1.0 - 0.85 * smoothstep(500.0, 4000.0, Depth * 0.01);
int r = (int)RippleCount;
float2 ripple = float2(0.0, 0.0);
for (int j = 0; j < r; j++)
{
	float4 c = WaveTex.Load(int3(MAX_WAVES + j, 0, 0));
	float a = c.x * d.x + c.y * d.y + WaveTex.Load(int3(MAX_WAVES + j, 1, 0)).x;
	ripple -= c.z * c.xy * sin(a);
}
slope += ripple * (1.0 - smoothstep(150.0, 900.0, Depth * 0.01));
return float4(normalize(float3(-slope.x, slope.y, 1.0)), eta);
)");
	}

	/** 水の色：波頭（H = 高さ [m]）ほど浅い色（光が透ける）、谷は深い色。江の島の海（ocean_drone_photoreal）の deep / shallow の考え方 */
	inline const TCHAR* ColorCode()
	{
		return TEXT("return lerp(Deep, Shallow, saturate(0.2 + 0.18 * H));");
	}

	/** 粗さ：遠くほど粗く（解像できない波の傾きのばらつき）。Base は近くの粗さ */
	inline const TCHAR* RoughnessCode()
	{
		return TEXT("return lerp(Base, 0.35, smoothstep(300.0, 5000.0, Depth * 0.01));");
	}

	/** 航海灯の射光範囲（Step 5）：Local = 灯火から視点への向き（部品の座標＝船体の座標、Unreal の X 前・Y 右）。
	 *  方位 = 船首から右回り [deg]。Center ± Half の内側で 1（端は ±0.5° で落とす）。Half >= 180 は全周。COLREG 第 21 条の射光範囲 */
	inline const TCHAR* NavSectorCode()
	{
		return TEXT(R"(
if (Half >= 180.0) { return 1.0; }
float b = degrees(atan2(Local.y, Local.x));
float d = abs(fmod(b - Center + 540.0, 360.0) - 180.0);
return saturate(Half + 0.5 - d);
)");
	}

	/** 空（Step 5）：Col = パノラマの色。昼は太陽、夜は月の円盤（DiskDir は Unreal の向き、DiskCos = cos(視半径)）と、夜の星（Stars > 0）。
	 *  返り値は輝度 [cd/m²]（SkyNits・Tint を掛けたパノラマ＋円盤＋星） */
	inline const TCHAR* SkyColorCode()
	{
		return TEXT(R"(
float3 v = -normalize(CamVec);
float3 sky = Col * SkyNits * Tint;
float c = dot(v, normalize(DiskDir));
float disk = smoothstep(DiskCos - 4e-6, DiskCos + 4e-6, c);
float3 stars = 0;
if (Stars > 0.0 && v.z > 0.02)
{
	float3 g = floor(v * 900.0);
	float h = frac(sin(dot(g, float3(12.9898, 78.233, 37.719))) * 43758.5453);
	float k = frac(h * 7.31);
	stars = (h > 0.9975 ? (0.2 + 0.8 * k * k) : 0.0) * Stars * smoothstep(0.02, 0.15, v.z);
}
return sky + disk * DiskNits * DiskColor + stars;
)");
	}

	/** 外側の平面：波面メッシュと重なる内側（半径 Hole [m]）は描かない（OpacityMask） */
	inline const TCHAR* FarMaskCode()
	{
		return TEXT("return length(Rel.xy) * 0.01 > Hole ? 1.0 : 0.0;");
	}

	/** 空の球：視線の向き（CamVec = CameraVector は画素からカメラへ。Unreal の世界）→ 正距円筒の UV
	 *  （tools/make_sky_hdri.py の pixel_dir と同じ式。東 = X、北 = −Y） */
	inline const TCHAR* SkyUvCode()
	{
		return TEXT(R"(
float3 v = -normalize(CamVec);
float el = asin(clamp(v.z, -1.0, 1.0));
float az = atan2(-v.y, v.x);
return float2(frac(az / 6.283185307 + 1.0), 0.5 - el / 3.141592654);
)");
	}
}
