#!/usr/bin/env python3
"""海の場面の空（SHP-24 Step 3）。江の島の Godot シーンと同じ Poly Haven の空パノラマ（CC0）から、
UE に取り込む縮小版と太陽の向きを作る。

  py -3.12 tools/make_sky_hdri.py [--src <4k の .exr>]

出力（コミット対象）:
  SourceArt/Sky/sky_2k.exr   正距円筒（2048×1024、half）。太陽の円盤は周りの空の明るさに抑える（太陽は平行光で描く）
  SourceArt/Sky/sky.json     太陽の向き（ENU）・元の画像・抑えた画素数

画素 (行 r, 列 c) の向き（ENU）: 仰角 el = π/2 − (r + 0.5)/H·π、方位 az = 2π·(c + 0.5)/W（東から反時計回り）、
d = (cos el·cos az, cos el·sin az, sin el)。空の材質（HakoniwaShipEditor の空の球）も同じ式で引く。
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")
import cv2  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRC = Path(r"D:/work_godot/3DMode_enoshima_PC/Materials/kloofendal_48d_partly_cloudy_puresky_4k.exr")
OUT_W, OUT_H = 2048, 1024
SUN_CLAMP = 40.0      # これより明るい画素（太陽の円盤とまわり）を周りの空の明るさに抑える


def pixel_dir(r, c, h, w):
    el = math.pi / 2 - (r + 0.5) / h * math.pi
    az = 2 * math.pi * (c + 0.5) / w
    return [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC)
    a = ap.parse_args()
    im = cv2.imread(str(a.src), cv2.IMREAD_UNCHANGED)
    if im is None:
        print(f"cannot read {a.src}", file=sys.stderr)
        return 1
    im = im[..., :3].astype(np.float32)   # BGR
    h, w = im.shape[:2]
    lum = im[..., 2] * 0.2126 + im[..., 1] * 0.7152 + im[..., 0] * 0.0722
    # 太陽：明るい画素の明るさ重みの重心（向きの平均）
    rr, cc = np.nonzero(lum > SUN_CLAMP)
    if len(rr) == 0:
        print("no sun found", file=sys.stderr)
        return 1
    wsum = np.zeros(3)
    for r, c in zip(rr, cc):
        wsum += lum[r, c] * np.array(pixel_dir(r, c, h, w))
    sun = wsum / np.linalg.norm(wsum)
    # 太陽を抑える：明るい画素を、そのまわり（明るい画素を除く）の平均色に置き換える
    mask = (lum > SUN_CLAMP).astype(np.uint8)
    ring = cv2.dilate(mask, np.ones((61, 61), np.uint8)) - cv2.dilate(mask, np.ones((21, 21), np.uint8))
    fill = im[ring > 0].mean(axis=0)
    grown = cv2.dilate(mask, np.ones((21, 21), np.uint8)) > 0
    # 抑えた分の放射照度（太陽に垂直な面、画像の単位）。UE では平行光の照度と空の輝度の比をこれでそろえる
    rows = np.nonzero(grown)[0]
    el_rows = math.pi / 2 - (rows + 0.5) / h * math.pi
    d_omega = (2 * math.pi / w) * (math.pi / h) * np.cos(el_rows)
    clipped = im[grown] - np.minimum(im[grown], fill * 1.5)
    sun_rgb = (clipped * d_omega[:, None]).sum(axis=0)[::-1]          # RGB
    sun_irr = float(sun_rgb @ np.array([0.2126, 0.7152, 0.0722]))
    im[grown] = np.minimum(im[grown], fill * 1.5)
    # 水平線の空の色（仰角 0〜3°、線形 RGB）：霧の色に使う
    r0, r1 = int(h * (0.5 - 3 / 180)), int(h * 0.5)
    horizon = im[r0:r1].reshape(-1, 3).mean(axis=0)[::-1]
    small = cv2.resize(im, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)
    out_dir = ROOT / "SourceArt" / "Sky"
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(out_dir / "sky_2k.exr"), small,
                     [cv2.IMWRITE_EXR_TYPE, cv2.IMWRITE_EXR_TYPE_HALF, cv2.IMWRITE_EXR_COMPRESSION, cv2.IMWRITE_EXR_COMPRESSION_ZIP])
    if not ok:
        print("cannot write exr", file=sys.stderr)
        return 1
    el = math.degrees(math.asin(sun[2]))
    az = math.degrees(math.atan2(sun[1], sun[0]))
    info = {
        "source": a.src.name,
        "license": "CC0 (Poly Haven)",
        "pixel_dir": "el = pi/2 - (r+0.5)/H*pi, az = 2*pi*(c+0.5)/W (ENU, from east counter-clockwise)",
        "sun_dir_enu": [round(float(v), 6) for v in sun],
        "sun_elevation_deg": round(el, 2),
        "sun_azimuth_enu_deg": round(az, 2),
        "clamped_pixels_4k": int(grown.sum()),
        "sun_irradiance": round(sun_irr, 4),
        "sun_rgb_norm": [round(float(v / sun_rgb.max()), 4) for v in sun_rgb],
        "horizon_rgb": [round(float(v), 4) for v in horizon],
        "size": [OUT_W, OUT_H],
    }
    (out_dir / "sky.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    print(f"sky: sun el {el:.1f} deg az {az:.1f} deg (ENU), clamped {int(grown.sum())} px -> {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
