#!/usr/bin/env python3
"""Unreal ビューアの姿勢の読み戻し（-ShipSamples）を、物理（ship_plant --trace）と照合する（SHP-24 Step 2・3）。

どちらも列は t,robot,x,y,z,roll,ang_y,yaw（pos PDU の値、D14・D18）。Unreal 側の値はアクタの変換から
ShipFrame の逆変換で読み戻したもので、t は表示時刻（物理時刻の軸。AShipFleet は受信値を表示時刻で線形補間して描く）。
そこで trace を同じ式（位置は線形、角度は差を ±π に折り返して線形）で t に補間して比べる。
Unreal は姿勢と stamp を同じフレームで続けて読むので、その間に物理が 1 ステップ進むことがある。
そのため補間する時刻を ±1 ステップずらしたものも試し、最も近いものを採ってずれたステップ数も数える。

usage: py -3.12 tools/compare_plant_trace.py <plant_trace.csv> <ue_samples.csv> [--min-samples 20]
成功時 HAKONIWA_UNREAL_SHIP_TRACE_OK を出して 0 で終わる。
"""
import argparse
import bisect
import csv
import math
import sys
from collections import defaultdict


def rot(roll, ang_y, yaw):
    """R = Rz(yaw)·Ry(-ang_y)·Rx(roll)（ship_pdu_io.hpp）"""
    p = -ang_y
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rot_angle(a, b):
    """二つの回転の差の角度 [rad]（tr(AᵀB) から）"""
    tr = sum(a[k][i] * b[k][i] for i in range(3) for k in range(3))
    return math.acos(max(-1.0, min(1.0, (tr - 1.0) * 0.5)))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def interp(tr, times, t):
    """trace を時刻 t に補間（AShipFleet::ApplyAt と同じ式）。範囲外は None"""
    i = bisect.bisect_right(times, t) - 1
    if i < 0 or i + 1 >= len(tr):
        return None
    a, b = tr[i], tr[i + 1]
    span = b[0] - a[0]
    u = min(1.0, max(0.0, (t - a[0]) / span)) if span > 0 else 1.0
    pos = [a[k] + u * (b[k] - a[k]) for k in (1, 2, 3)]
    ang = [a[k] + u * wrap(b[k] - a[k]) for k in (4, 5, 6)]
    return pos, ang


def load(path):
    rows = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            # ship_plant はテストの終わりに止められるので、trace の最後の行が途中で切れていることがある
            if any(r.get(k) in (None, "") for k in ("t", "robot", "x", "y", "z", "roll", "ang_y", "yaw")):
                continue
            rows[r["robot"]].append(tuple(float(r[k]) for k in ("t", "x", "y", "z", "roll", "ang_y", "yaw")))
    for v in rows.values():
        v.sort()
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("samples")
    ap.add_argument("--window", type=float, default=0.061, help="照合する trace の時刻の幅 ± [s]（20 ms ステップの ±3）")
    ap.add_argument("--min-samples", type=int, default=20)
    ap.add_argument("--pos-tol", type=float, default=1.0e-3)
    ap.add_argument("--ang-tol", type=float, default=1.0e-4)
    a = ap.parse_args()
    trace = load(a.trace)
    samples = load(a.samples)
    problems = []
    for robot in sorted(set(trace) | set(samples)):
        tr, sm = trace.get(robot, []), samples.get(robot, [])
        if len(sm) < a.min_samples:
            problems.append(f"{robot}: {len(sm)} samples < {a.min_samples}")
            continue
        times = [r[0] for r in tr]
        worst_pos = worst_ang = 0.0
        offsets = defaultdict(int)
        # ship_plant はテストの終わりに止められ、trace の末尾（バッファの中身）が書かれないことがある。
        # trace の終わりより後の標本は照合できないので除き、数だけ出す（照合した数が --min-samples 以上であること）
        # また表示時刻は最新の物理時刻より少し遅れて始まるので、最初の標本が trace の始まりより前のことがある（これも除く）
        end = times[-1] - a.window if times else float("-inf")
        start = times[0] + a.window if times else float("inf")
        skipped = sum(1 for s in sm if s[0] > end or s[0] < start)
        sm = [s for s in sm if start <= s[0] <= end]
        if len(sm) < a.min_samples:
            problems.append(f"{robot}: {len(sm)} comparable samples < {a.min_samples} (skipped {skipped} after trace end)")
            continue
        for s in sm:
            rs = rot(*s[4:])
            best = None
            for k in (-1, 0, 1):
                r = interp(tr, times, s[0] + k * 0.02)
                if r is None:
                    continue
                dp = math.dist(s[1:4], r[0])
                da = rot_angle(rs, rot(*r[1]))
                if best is None or dp + da < best[0] + best[1]:
                    best = (dp, da, k)
            if best is None:
                problems.append(f"{robot}: no trace rows around t={s[0]:.3f}")
                continue
            worst_pos = max(worst_pos, best[0])
            worst_ang = max(worst_ang, best[1])
            offsets[best[2]] += 1
        print(f"  {robot}: samples {len(sm)}  t {sm[0][0]:.2f}..{sm[-1][0]:.2f} s  max pos err {worst_pos:.2e} m  "
              f"max rot err {worst_ang:.2e} rad  matched step offsets {dict(sorted(offsets.items()))}  skipped outside trace {skipped}")
        if worst_pos > a.pos_tol:
            problems.append(f"{robot}: pos err {worst_pos:.3e} m > {a.pos_tol}")
        if worst_ang > a.ang_tol:
            problems.append(f"{robot}: rot err {worst_ang:.3e} rad > {a.ang_tol}")
    if problems:
        print("HAKONIWA_UNREAL_SHIP_TRACE_FAILED " + "; ".join(problems))
        return 1
    print("HAKONIWA_UNREAL_SHIP_TRACE_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
