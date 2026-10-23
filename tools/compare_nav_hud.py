#!/usr/bin/env python3
"""ブリッジ視界・FOC に出した航海情報（-ShipHudLog）を、物理の trace（ship_plant --trace）から独立に求めた値と比べる（SHP-25 Step 2）。

  python compare_nav_hud.py <plant_trace.csv> <hud_log.csv> [--window 5.0]

HUD のログは 1 s ごとの表示時刻 t の値。ここでは trace の行のうち t − window ≤ 行の時刻 ≤ t のものから、Godot の _fit_velocity と同じ最小二乗で
速度（東・北）・回頭角速度を求め、SOG/COG・HDG（最後の行の yaw）、他船の距離・方位・CPA/TCPA・衝突の危険（cpa_ozt.hpp の compute_cpa と同じ式・閾値
926 m / 900 s）を計算して比べる。Unreal は物理の受信値（PDU を読み損ねた時刻は欠ける）で同じ計算をしているので、差は小さいはず。
始まりの window + 1 s は比べない（開始直後は物理が遅れを取り戻すため数ステップずつ進み、Unreal が受け取れない標本があって窓の中身が違う）。
成功で HAKONIWA_UNREAL_SHIP_NAV_OK。
"""
import argparse
import bisect
import csv
import math
import sys
from collections import defaultdict

KT = 1.0 / 0.514444
NM = 1.0 / 1852.0
CPA_ALARM_M, TCPA_ALARM_S = 926.0, 900.0
TOL = {"e": 0.05, "n": 0.05, "hdg_deg": 0.05, "sog_kt": 0.03, "cog_deg": 0.3, "rot_deg_min": 0.3,
       "range_nm": 1e-4, "bearing_deg": 0.01, "cpa_nm": 0.01, "tcpa_min": 0.05}


def load_trace(path):
    rows = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if any(r.get(k) in (None, "") for k in ("t", "robot", "x", "y", "yaw")):
                continue
            rows[r["robot"]].append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["yaw"])))
    for v in rows.values():
        v.sort()
    return rows


def wrap360(d):
    return d % 360.0


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def estimate(rows, times, t, window):
    last = bisect.bisect_right(times, t + 1e-9) - 1
    if last < 0:
        return None
    first = last
    while first > 0 and t - rows[first - 1][0] <= window + 1e-9:
        first -= 1
    seg = rows[first:last + 1]
    yaw = [seg[0][3]]
    for i in range(1, len(seg)):
        d = seg[i][3] - seg[i - 1][3]
        yaw.append(yaw[-1] + math.atan2(math.sin(d), math.cos(d)))
    n = len(seg)
    mt = sum(s[0] for s in seg) / n
    me = sum(s[1] for s in seg) / n
    mn = sum(s[2] for s in seg) / n
    my = sum(yaw) / n
    stt = sum((s[0] - mt) ** 2 for s in seg)
    ve = sum((s[0] - mt) * (s[1] - me) for s in seg) / stt if stt > 0 else 0.0
    vn = sum((s[0] - mt) * (s[2] - mn) for s in seg) / stt if stt > 0 else 0.0
    vy = sum((s[0] - mt) * (yaw[i] - my) for i, s in enumerate(seg)) / stt if stt > 0 else 0.0
    e, nn, y = seg[-1][1], seg[-1][2], seg[-1][3]
    return {"e": e, "n": nn, "ve": ve, "vn": vn, "hdg_deg": wrap360(90.0 - math.degrees(y)),
            "sog_kt": math.hypot(ve, vn) * KT, "cog_deg": wrap360(math.degrees(math.atan2(ve, vn))),
            "rot_deg_min": -math.degrees(vy) * 60.0, "samples": n}


def cpa(own, tgt):
    pe, pn = tgt["e"] - own["e"], tgt["n"] - own["n"]
    ve, vn = tgt["ve"] - own["ve"], tgt["vn"] - own["vn"]
    v2 = ve * ve + vn * vn
    if v2 < 1e-4:
        d = math.hypot(pe, pn)
        return d * NM, 0.0, d <= CPA_ALARM_M
    tc = -(pe * ve + pn * vn) / v2
    d = math.hypot(pe + ve * tc, pn + vn * tc)
    return d * NM, tc / 60.0, tc > 0.0 and tc <= TCPA_ALARM_S and d <= CPA_ALARM_M


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("hud")
    ap.add_argument("--window", type=float, default=5.0)
    a = ap.parse_args()
    trace = load_trace(a.trace)
    times = {k: [r[0] for r in v] for k, v in trace.items()}
    hud = list(csv.DictReader(open(a.hud, encoding="utf-8")))
    own_robot = hud[0]["robot"] if hud else None
    worst = defaultdict(float)
    risk_mismatch, compared, skipped = 0, 0, 0
    end = min(v[-1][0] for v in trace.values()) if trace else 0.0
    settle = (float(hud[0]["t"]) if hud else 0.0) + a.window + 1.0
    for r in hud:
        t = float(r["t"])
        if t < settle or t > end - 0.1:
            skipped += 1   # 始まり（上）と trace の終わり（ship_plant を止めたとき最後の行が書かれないことがある）
            continue
        st = estimate(trace[r["robot"]], times[r["robot"]], t, a.window)
        if st is None:
            continue
        compared += 1
        for k in ("e", "n", "sog_kt", "rot_deg_min"):
            worst[k] = max(worst[k], abs(float(r[k]) - st[k]))
        worst["hdg_deg"] = max(worst["hdg_deg"], abs(wrap180(float(r["hdg_deg"]) - st["hdg_deg"])))
        if st["sog_kt"] > 0.5:   # 止まっているときの COG は決まらない
            worst["cog_deg"] = max(worst["cog_deg"], abs(wrap180(float(r["cog_deg"]) - st["cog_deg"])))
        if r["robot"] != own_robot and r["range_nm"]:
            own = estimate(trace[own_robot], times[own_robot], t, a.window)
            de, dn = st["e"] - own["e"], st["n"] - own["n"]
            worst["range_nm"] = max(worst["range_nm"], abs(float(r["range_nm"]) - math.hypot(de, dn) * NM))
            worst["bearing_deg"] = max(worst["bearing_deg"], abs(wrap180(float(r["bearing_deg"]) - math.degrees(math.atan2(de, dn)))))
            c, tc, risk = cpa(own, st)
            worst["cpa_nm"] = max(worst["cpa_nm"], abs(float(r["cpa_nm"]) - c))
            worst["tcpa_min"] = max(worst["tcpa_min"], abs(float(r["tcpa_min"]) - tc))
            risk_mismatch += int(int(r["risk"]) != int(risk))
    problems = [f"{k} err {worst[k]:.4g} > {TOL[k]}" for k in TOL if worst[k] > TOL[k]]
    if risk_mismatch:
        problems.append(f"collision risk flag differs in {risk_mismatch} rows")
    if compared < 10:
        problems.append(f"only {compared} rows compared")
    print("  nav compared %d rows (skipped %d at start / after trace end): " % (compared, skipped)
          + ", ".join(f"{k} {worst[k]:.3g}" for k in TOL) + f", risk flag mismatches {risk_mismatch}")
    if problems:
        print("HAKONIWA_UNREAL_SHIP_NAV_FAILED " + "; ".join(problems))
        return 1
    print("HAKONIWA_UNREAL_SHIP_NAV_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
