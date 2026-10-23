// ShipFrame.h
// 座標の約束（1 か所にまとめる）。
//   船舶 PDU の世界（D14：pos.linear は FLU/ENU）：右手系、m、z 上（世界は ENU の x 東・y 北、船体は FLU の x 前・y 左）
//   Unreal：左手系、cm、X 前・Y 右・Z 上
//   → Unreal = (100·x, −100·y, 100·z)。向き（単位ベクトル）は (x, −y, z)。hakoniwa-unreal-rover の RoverFrame と同じ。
//   姿勢：pos.angular = (roll, asin(R20), yaw)。roll は右手系（D18）、y は右手系ピッチの符号反転（D14）、yaw は東から反時計回り。
//         R = Rz(yaw)·Ry(−ang_y)·Rx(roll)（hakoniwa-mujoco-ship の ship_pdu_io.hpp と同じ）。
//   回転の写像：Unreal の座標 = S·右手系の座標（S = diag(1, −1, 1)）なので、回転は S·R·S（rover の FluRotationToUnreal と同じ）。

#pragma once

#include "CoreMinimal.h"

namespace ShipFrame
{
	inline FVector LocalToUnreal(double X, double Y, double Z) { return FVector(X * 100.0, -Y * 100.0, Z * 100.0); }
	inline FVector LocalToUnreal(const FVector& L) { return LocalToUnreal(L.X, L.Y, L.Z); }
	inline FVector LocalDirToUnreal(double X, double Y, double Z) { return FVector(X, -Y, Z).GetSafeNormal(); }
	inline FVector UnrealToLocal(const FVector& U) { return FVector(U.X / 100.0, -U.Y / 100.0, U.Z / 100.0); }

	/** 右手系の 3×3 回転（列ベクトルの約束・R[行][列]）。FMatrix の M[i][j] を R[i][j] として使う（4 行目・4 列目は単位） */
	inline FMatrix RotX(double A) { const double c = FMath::Cos(A), s = FMath::Sin(A); FMatrix M = FMatrix::Identity; M.M[1][1] = c; M.M[1][2] = -s; M.M[2][1] = s; M.M[2][2] = c; return M; }
	inline FMatrix RotY(double A) { const double c = FMath::Cos(A), s = FMath::Sin(A); FMatrix M = FMatrix::Identity; M.M[0][0] = c; M.M[0][2] = s; M.M[2][0] = -s; M.M[2][2] = c; return M; }
	inline FMatrix RotZ(double A) { const double c = FMath::Cos(A), s = FMath::Sin(A); FMatrix M = FMatrix::Identity; M.M[0][0] = c; M.M[0][1] = -s; M.M[1][0] = s; M.M[1][1] = c; return M; }
	inline FMatrix Mul(const FMatrix& A, const FMatrix& B)   // 列ベクトルの約束で A·B
	{
		FMatrix R = FMatrix::Identity;
		for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
		{
			double v = 0.0;
			for (int k = 0; k < 3; ++k) v += A.M[i][k] * B.M[k][j];
			R.M[i][j] = v;
		}
		return R;
	}
	/** 単位軸 Axis（右手系）まわりに A [rad] の回転（Rodrigues） */
	inline FMatrix RotAxis(const FVector& Axis, double A)
	{
		const FVector n = Axis.GetSafeNormal();
		const double c = FMath::Cos(A), s = FMath::Sin(A), t = 1.0 - c;
		FMatrix M = FMatrix::Identity;
		M.M[0][0] = t * n.X * n.X + c;       M.M[0][1] = t * n.X * n.Y - s * n.Z; M.M[0][2] = t * n.X * n.Z + s * n.Y;
		M.M[1][0] = t * n.X * n.Y + s * n.Z; M.M[1][1] = t * n.Y * n.Y + c;       M.M[1][2] = t * n.Y * n.Z - s * n.X;
		M.M[2][0] = t * n.X * n.Z - s * n.Y; M.M[2][1] = t * n.Y * n.Z + s * n.X; M.M[2][2] = t * n.Z * n.Z + c;
		return M;
	}

	/** pos.angular（D14・D18）→ 右手系の回転 R = Rz(yaw)·Ry(−ang_y)·Rx(roll) */
	inline FMatrix PduAngularToRotation(double Roll, double AngY, double Yaw) { return Mul(RotZ(Yaw), Mul(RotY(-AngY), RotX(Roll))); }

	/** 右手系の回転 → pos.angular（ship_pdu_io.hpp の pose_from_xmat と同じ式） */
	inline FVector RotationToPduAngular(const FMatrix& R)
	{
		const double AngY = FMath::Asin(FMath::Clamp(R.M[2][0], -1.0, 1.0));   // R20 = −sin(pitch) なので = −pitch（D14）
		if (FMath::Abs(FMath::Cos(AngY)) > 1e-6)
		{
			return FVector(FMath::Atan2(R.M[2][1], R.M[2][2]), AngY, FMath::Atan2(R.M[1][0], R.M[0][0]));
		}
		return FVector(0.0, AngY, FMath::Atan2(-R.M[0][1], R.M[1][1]));
	}

	/** 右手系の回転 → Unreal の回転（S·R·S。FMatrix は行ベクトルの約束（v' = v·M）なので転置して入れる） */
	inline FQuat RotationToUnreal(const FMatrix& R)
	{
		static const double S[3] = { 1.0, -1.0, 1.0 };
		FMatrix M = FMatrix::Identity;
		for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
		{
			M.M[j][i] = S[i] * S[j] * R.M[i][j];
		}
		return FQuat(M);
	}

	/** Unreal の回転 → 右手系の回転（RotationToUnreal の逆。検証で使う） */
	inline FMatrix UnrealToRotation(const FQuat& Q)
	{
		static const double S[3] = { 1.0, -1.0, 1.0 };
		const FMatrix M = FRotationMatrix::Make(Q);
		FMatrix R = FMatrix::Identity;
		for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
		{
			R.M[i][j] = S[i] * S[j] * M.M[j][i];
		}
		return R;
	}
}
