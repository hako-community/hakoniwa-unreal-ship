#!/usr/bin/env python3
"""船載カメラの画像の水平線を、同じ時刻の船の姿勢（ship_plant --trace）から求めた位置と比べる（SHP-25）。

  python compare_camera_horizon.py <plant_trace.csv> <fleet_ue.json> <camera_dir> [--camera camera_bridge_front]

カメラは船体に固定で船首方向・水平（fleet_ue.json の cameras）。姿勢 R = Rz(yaw)·Ry(−ang_y)·Rx(roll)（D14・D18）のとき、
画素 (u, v) の視線は FLU で d = (1, −(u−cx)/f, −(v−cy)/f)、世界の上下成分 (R·d)_z = 0 が水平線（海は平面なので仰角 0）：
  v = cy + f·(R20 − R21·(u−cx)/f) / R22、f = (W/2) / tan(水平画角/2)
画像の左右の端の列（積荷・船体が写らない）で、上の空（明るい）と下の海（暗い）の境目を探し、求めた行と比べる。
画像の時刻（header.stamp）の姿勢は trace を線形補間する。境目はきめ（海の波）で見つけるため波面メッシュの外周になり、真の水平線より一定だけ下
（走行全体の中央値として引く）。合否：残差の中央値 1 画素以内・最大 4 画素以内（波面メッシュはメインのカメラを中心に置くので、外周までの距離が向きで少し違い、
列によって ±3 画素ほど揺れる）、一定のずれ 0〜6 画素で CAMERA_HORIZON_OK。
（画像と時刻の取り違え（0.1 s）・横揺れの符号・取り付けの向きの誤りはここで数画素〜十数画素の差になる）
"""
import argparse
import bisect
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

EDGE_COLS = 60   # 左右の端から何列を使うか


def load_trace(path, robot):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("robot") != robot or any(r.get(k) in (None, "") for k in ("t", "roll", "ang_y", "yaw")):
                continue
            rows.append((float(r["t"]), float(r["roll"]), float(r["ang_y"]), float(r["yaw"])))
    rows.sort()
    return rows


def pose_at(trace, times, t):
    i = bisect.bisect_right(times, t) - 1
    if i < 0 or i + 1 >= len(trace):
        return None
    a, b = trace[i], trace[i + 1]
    u = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    wrap = lambda x: math.atan2(math.sin(x), math.cos(x))
    return [a[k] + u * wrap(b[k] - a[k]) for k in (1, 2, 3)]


def expected_row(roll, ang_y, u, cx, cy, f):
    r20 = math.sin(ang_y)                      # R20 = sin(ang_y)（D14）
    r21 = math.cos(ang_y) * math.sin(roll)
    r22 = math.cos(ang_y) * math.cos(roll)
    return cy + f * (r20 - r21 * (u - cx) / f) / r22


SEARCH_UP, SEARCH_DOWN = 40, 60  # 画像の中央の行から上へ・下へ探す範囲（雲を拾わない。船の揺れでの水平線の動きは ±20 画素程度）


ROUGH_THRESHOLD = 5.5   # 横に隣り合う画素の明るさの差の平均。空 1〜2、雲の縁 5 前後、海（波・きらめき）7 以上


def measured_row(img, u):
    """空と海の境目の行。明るさでは決まらない（遠くの海は空を映して霞とほぼ同じ明るさ）ので、きめで決める：
    列 u ± 8 の各行で横に隣り合う画素の明るさの差の平均（粗さ）を求め、探す範囲（画像の中央から上へ SEARCH_UP・下へ SEARCH_DOWN）で
    上から見て最初に粗さが ROUGH_THRESHOLD を超え、その下 10 行の中央値も超える行。期待値のまわりでは探さない。
    返り値は（行、その下 10 行の粗さの中央値）"""
    lum = img.astype(np.float64) @ np.array([0.299, 0.587, 0.114])
    blk = lum[:, max(0, u - 8):u + 9]
    rough = np.abs(np.diff(blk, axis=1)).mean(axis=1)
    c = len(rough) // 2
    for v in range(c - SEARCH_UP, c + SEARCH_DOWN):
        if rough[v] > ROUGH_THRESHOLD and float(np.median(rough[v:v + 10])) > ROUGH_THRESHOLD:
            return v - 0.5, float(np.median(rough[v:v + 10]))   # 境目は行 v−1 と v の間
    return float(c), 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("fleet")
    ap.add_argument("camera_dir")
    ap.add_argument("--camera", default="camera_bridge_front")
    ap.add_argument("--median-tol", type=float, default=1.0)
    ap.add_argument("--max-tol", type=float, default=4.0)
    a = ap.parse_args()
    fleet = json.load(open(a.fleet, encoding="utf-8"))
    cam = next(c for c in fleet["cameras"] if c["frame_id"] == a.camera)
    W, H = cam["width"], cam["height"]
    f = (W / 2) / math.tan(math.radians(cam["fov_deg"]) / 2)
    cx, cy = (W - 1) / 2, (H - 1) / 2
    trace = load_trace(a.trace, cam["robot"])
    times = [r[0] for r in trace]
    errs, rolls, used = [], [], 0
    for p in sorted(Path(a.camera_dir).glob(f"{a.camera}_*.npy")):
        stamp = float(p.stem[len(a.camera) + 1:])
        if stamp < 1.0:
            continue   # 始まって 1 s までは海面がまだ描かれていないことがある（trace の照合と同じく除く）
        pose = pose_at(trace, times, stamp)
        if pose is None:
            continue
        img = np.load(p)
        roll, ang_y, _ = pose
        cols = list(range(2, EDGE_COLS, 4)) + list(range(W - EDGE_COLS, W - 2, 4))
        e = []
        for u in cols:
            want = expected_row(roll, ang_y, u, cx, cy, f)
            got, contrast = measured_row(img, u)
            if contrast <= 0.0:
                continue   # 境目が見つからない（夜など）
            e.append(got - want)
        if not e:
            continue
        used += 1
        errs.extend(e)
        rolls.append(roll)
        print(f"  {p.name}: roll {math.degrees(roll):+.2f} deg, ang_y {math.degrees(ang_y):+.2f} deg, "
              f"horizon err median {np.median(e):+.2f} px, max |{np.max(np.abs(e)):.2f}| px over {len(e)} columns")
    if used < 5:
        print(f"CAMERA_HORIZON_FAILED only {used} images compared")
        return 1
    # 見つけるのは海のきめの始まり＝波面メッシュの外周（約 1.2 km 先、外側の平面は法線が平らで滑らか）で、真の水平線より
    # 俯角 atan(高さ / 距離) だけ下（船橋のカメラで 0.75°・約 3 画素）。この一定のずれを走行全体の中央値として引き、残り（列ごとの傾き＝
    # 横揺れ、画像ごとの上下＝縦揺れ・時刻）で判定する。ずれ自体も 0〜6 画素であること
    offset = float(np.median(errs))
    res = np.array(errs) - offset
    med, mx = float(np.median(np.abs(res))), float(np.max(np.abs(res)))
    print(f"  {a.camera}: {used} images, constant offset {offset:+.2f} px (wave-mesh edge below the horizon), "
          f"residual |err| median {med:.2f} px, max {mx:.2f} px "
          f"(1 px = {math.degrees(1 / f):.3f} deg; roll range {math.degrees(min(rolls)):+.2f}..{math.degrees(max(rolls)):+.2f} deg)")
    if med > a.median_tol or mx > a.max_tol or not (0.0 <= offset <= 6.0):
        print(f"CAMERA_HORIZON_FAILED residual median {med:.2f} px (tol {a.median_tol}), max {mx:.2f} px (tol {a.max_tol}), offset {offset:+.2f} px (0..6)")
        return 1
    print("CAMERA_HORIZON_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
