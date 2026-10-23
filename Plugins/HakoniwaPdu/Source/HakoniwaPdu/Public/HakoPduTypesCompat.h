#pragma once

// ★ 2026-10-03（related/devai_dll/hakodrone_unreal_drone_migration_20260930.md・A1）:
//   Android だけで、HakoniwaPdu・HakoniwaDrone の Build.cs が全部の翻訳単位の先頭に読ませる（ForceIncludeFiles）。
//
//   hakoniwa-pdu-registry（外部のサブモジュール・中身は変えない）の ros_primitive_types.hpp は
//   int8〜uint64・float32・float64 を全体の名前で typedef する。Unreal も同じ名前の int64・uint64 を持ち、
//   Windows ではどちらも long long なので衝突しないが、Android（LP64）では int64_t が long になって
//   「型の違う再定義」になる（サイズはどちらも 64 ビット）。
//   → そのヘッダの二重読み込み防止の印を先に立てて中身を読ませず、int8〜uint64 は Unreal のものを使う。
//     Unreal に無い float32・float64 だけをここで補う。
#ifndef _ROS_PRIMITIVE_TYPES_HPP_
#define _ROS_PRIMITIVE_TYPES_HPP_
#endif

typedef float float32;
typedef double float64;
