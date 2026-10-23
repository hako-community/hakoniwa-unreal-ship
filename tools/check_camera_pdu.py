#!/usr/bin/env python3
"""船載カメラの PDU（SHP-25）を受け手として読む箱庭アセット。run_plant_test.ps1 が SDK の Python（hakopy）で起動する。

  python check_camera_pdu.py <pdu_config_ship.json> <fleet_ue.json> <out_dir> --duration <物理時刻 s>

fleet_ue.json の cameras の PDU を毎ステップ読み（hakopy.pdu_read、共有メモリの今の中身）、header.stamp が変わったら新しい画像として
  out_dir/camera_frames.csv  … 受け取った画像ごと：camera, stamp, frame_id, width, height, encoding, step, data_len, mean_luma
  out_dir/<frame_id>_<stamp>.npy … 2 s ごとの画像（rgb8、tools/compare_camera_horizon.py が物理の trace と照合する）
を書き、--duration の後に受け取りの検証（数・時刻の順・間隔・形式）を CAMERA_PDU_OK / CAMERA_PDU_FAILED で出す。
その後も止められるまで時刻を進め続ける（箱庭は全アセットの足並みをそろえるので、抜けるとシミュレーション全体が止まる）。
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import hakopy
import numpy as np
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_Image import pdu_to_py_Image

ASSET_NAME = "ShipCameraCheck"
DELTA_USEC = 20000
SAVE_EVERY_S = 2.0

_state = {}


def on_manual_timing_control(_context):
    s = _state
    cams = s["cams"]
    last = {c["pdu"]: None for c in cams}
    next_save = {c["pdu"]: 0.0 for c in cams}
    first_t = None
    with open(s["out"] / "camera_frames.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["camera", "stamp", "frame_id", "width", "height", "encoding", "step", "data_len", "mean_luma"])
        while True:
            t = hakopy.simulation_time() * 1e-6
            for c in cams:
                raw = hakopy.pdu_read(c["robot"], c["channel_id"], c["pdu_size"])
                if not raw:
                    continue
                try:
                    img = pdu_to_py_Image(raw)
                except Exception:   # まだ一度も書かれていない（0 埋め）
                    continue
                stamp = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9
                if stamp <= 0.0 or stamp == last[c["pdu"]]:
                    continue
                last[c["pdu"]] = stamp
                data = np.frombuffer(bytes(img.data), dtype=np.uint8)
                luma = float((data.reshape(-1, 3) @ np.array([0.299, 0.587, 0.114])).mean()) if data.size and data.size % 3 == 0 else -1.0
                w.writerow([c["pdu"], f"{stamp:.6f}", img.header.frame_id, img.width, img.height, img.encoding, img.step, len(img.data), f"{luma:.2f}"])
                if stamp >= next_save[c["pdu"]] and data.size == img.width * img.height * 3:
                    next_save[c["pdu"]] = stamp + SAVE_EVERY_S
                    np.save(s["out"] / f"{img.header.frame_id}_{stamp:09.3f}.npy", data.reshape(img.height, img.width, 3))
            if first_t is None and any(v is not None for v in last.values()):
                first_t = t
            if first_t is not None and t - first_t >= s["duration"]:
                break
            if not hakopy.usleep(DELTA_USEC):
                return 0
    print("CAMERA_CHECK_DONE", flush=True)
    s["result"] = summarize(s["out"], cams, s["duration"])
    sys.stdout.flush()
    # ★ 箱庭のアセットは足並みをそろえて進む（conductor が全アセットを待つ）。読み終えても時刻を進め続けないとシミュレーション全体が止まる
    while hakopy.usleep(DELTA_USEC):
        pass
    return 0


def _noop(_context):
    return 0


def summarize(out: Path, cams: list, duration: float) -> int:
    rows = list(csv.DictReader(open(out / "camera_frames.csv", encoding="utf-8")))
    problems = []
    for c in cams:
        r = [x for x in rows if x["camera"] == c["pdu"]]
        st = [float(x["stamp"]) for x in r]
        if len(r) < 0.9 * duration * c["rate_hz"]:
            problems.append(f"{c['pdu']}: {len(r)} images in {duration:.0f} s (expected ~{duration * c['rate_hz']:.0f})")
        if not r:
            continue
        bad = [x for x in r if (x["frame_id"], int(x["width"]), int(x["height"]), x["encoding"], int(x["step"]), int(x["data_len"]))
               != (c["frame_id"], c["width"], c["height"], "rgb8", c["width"] * 3, c["width"] * c["height"] * 3)]
        if bad:
            problems.append(f"{c['pdu']}: {len(bad)} images with wrong format (first: {dict(bad[0])})")
        gaps = np.diff(st)
        if len(gaps) and (gaps.min() <= 0.0):
            problems.append(f"{c['pdu']}: stamps not increasing")
        period = 1.0 / c["rate_hz"]
        # 区切りを越えた最初のフレームで撮るので、間隔は周期 ± 1 フレーム（描画 100 FPS 以上で 10 ms 未満）。受け手の読み落としは周期の倍
        late = int(np.sum(gaps > 1.5 * period)) if len(gaps) else 0
        luma = [float(x["mean_luma"]) for x in r]
        print(f"  {c['pdu']} ({c['frame_id']}): {len(r)} images, stamps {st[0]:.3f}..{st[-1]:.3f} s, "
              f"interval {gaps.mean() if len(gaps) else 0:.4f} s (min {gaps.min() if len(gaps) else 0:.4f}, max {gaps.max() if len(gaps) else 0:.4f}), "
              f"gaps > 1.5 periods {late}, mean luma {min(luma):.1f}..{max(luma):.1f}")
        if late > 0.02 * len(gaps):
            problems.append(f"{c['pdu']}: {late} gaps longer than 1.5 periods")
        if min(luma) < 5.0 or max(luma) > 250.0:
            problems.append(f"{c['pdu']}: image too dark or too bright (mean luma {min(luma):.1f}..{max(luma):.1f})")
    if problems:
        print("CAMERA_PDU_FAILED " + "; ".join(problems))
        return 1
    print("CAMERA_PDU_OK")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("pdu_def")
    ap.add_argument("fleet")
    ap.add_argument("out")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--summarize-only", action="store_true")
    a = ap.parse_args()
    fleet = json.load(open(a.fleet, encoding="utf-8"))
    pdus = {(p["robot"], p["pdu"]): p for p in fleet["pdus"]}
    cams = []
    for c in fleet.get("cameras", []):
        p = pdus[(c["robot"], c["pdu"])]
        cams.append({**c, "channel_id": p["channel_id"], "pdu_size": p["pdu_size"]})
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if a.summarize_only:
        return summarize(out, cams, a.duration)
    _state.update(cams=cams, out=out, duration=a.duration)
    callbacks = {"on_initialize": _noop, "on_simulation_step": None, "on_manual_timing_control": on_manual_timing_control, "on_reset": _noop}
    if not hakopy.asset_register(ASSET_NAME, a.pdu_def, callbacks, DELTA_USEC, hakopy.HAKO_ASSET_MODEL_CONTROLLER):
        print("CAMERA_CHECK_REGISTER_FAILED", flush=True)
        return 1
    print("CAMERA_CHECK_REGISTERED", flush=True)
    hakopy.start()
    return _state.get("result", 1)


if __name__ == "__main__":
    sys.exit(main())
