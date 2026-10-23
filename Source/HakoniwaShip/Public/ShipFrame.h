// ShipFrame.h
// 座標の約束（1 か所にまとめる）。
//   船舶 PDU の世界（D14：pos.linear は FLU/ENU）：右手系、m、z 上（世界は ENU の x 東・y 北、船体は FLU の x 前・y 左）
//   Unreal：左手系、cm、X 前・Y 右・Z 上
//   → Unreal = (100·x, −100·y, 100·z)。向き（単位ベクトル）は (x, −y, z)。hakoniwa-unreal-rover の RoverFrame と同じ。
//   姿勢（pos.angular = (roll, asin(R20), yaw)、roll は右手系＝D18）の写像は Step 2 で AShipActor と一緒に決める。

#pragma once

#include "CoreMinimal.h"

namespace ShipFrame
{
	inline FVector LocalToUnreal(double X, double Y, double Z) { return FVector(X * 100.0, -Y * 100.0, Z * 100.0); }
	inline FVector LocalDirToUnreal(double X, double Y, double Z) { return FVector(X, -Y, Z).GetSafeNormal(); }
	inline FVector UnrealToLocal(const FVector& U) { return FVector(U.X / 100.0, -U.Y / 100.0, U.Z / 100.0); }
}
