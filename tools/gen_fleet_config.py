#!/usr/bin/env python3
"""船団の設定（Godot ビューアと同じ船・PDU）から Unreal ビューア用の Config/Fleet/fleet_ue.json を作る（SHP-24 Step 2）。

入力（隣のリポジトリ。手で写すと Godot ビューア・物理とずれるため、ここから生成する）:
  ../hakoniwa-godot-ship/config/fleet.json                 船の一覧（robot・model・PDU 名・preview_pose・AIS）
  ../pdu_config_ship.json                                  PDU の表（型・チャネル・大きさ）
  ../hakoniwa-godot-ship/Models/<model>/<model>.glb        船体・可動部のノード（原点）とメッシュの外形
  ../hakoniwa-godot-ship/Models/<model>/ship_sync.profile.json   関節名・ノード名・回転軸（FLU）
  ../hakoniwa-godot-ship/tests/integration/plant_coupling/expected.json   結合テストの期待値（波浪・船の水線面の範囲・閾値、Step 3）
  ../hakoniwa-mujoco-ship/include/ship/environment/wave_spectrum.hpp     物理と同じ波面の計算（D4、Step 3）
  ../hakoniwa-mujoco-ship/include/ship/autonomy/cpa_ozt.hpp               CPA/TCPA/OZT の計算（SHP-25 Step 2。Godot の cpa_ozt.gd と同じ元）
  ../../images/image2sim-framework/output/<model>/resolved_config.json  航海灯（色・射光範囲・視認距離、Step 5）。
                                                                         物理（ship_plant の models_dir）と同じ船舶生成器の出力。glb のノードには載っていない

出力（コミット対象）:
  Config/Fleet/fleet_ue.json
  Source/HakoniwaShip/Public/ThirdParty/wave_spectrum.hpp（物理の wave_spectrum.hpp の写し。内容は変えない）
  Source/HakoniwaShip/Public/ThirdParty/cpa_ozt.hpp（物理の cpa_ozt.hpp の写し。内容は変えない）
  SourceArt/Ships/<model>.glb（Godot ビューアの glb の写し。場面づくり（ship_scene.ps1 -Build）が部品ごとの StaticMesh にする、Step 4）
  座標はすべて FLU（右手系 m、x 前・y 左・z 上）。glb（glTF、Y 上）からは x = -Z, y = -X, z = +Y で戻す
  （Godot の hakoniwa_robot_sync の x→-Z, y→-X, z→+Y の逆）。Unreal への写像は ShipFrame.h。
  parts / joints[].part は glb のメッシュのノード。origin はノードの原点（FLU）、box_min / box_max は外形の箱
  （船体の座標。関節の部品は関節の原点からの相対）。Unreal は glb から作った StaticMesh を置き、外形がこの箱と一致するか確かめる
  （メッシュが無いときは箱を仮の形として置く）。materials は glb の材質（色・金属度・粗さ・発光、Godot と同じ値）。

usage: py -3.12 tools/gen_fleet_config.py [--check]
"""
import argparse
import json
import struct

import numpy as np
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
SHIPNAVI = PROJECT.parent
GODOT = SHIPNAVI / "hakoniwa-godot-ship"
OUT = PROJECT / "Config" / "Fleet" / "fleet_ue.json"
WAVE_SRC = SHIPNAVI / "hakoniwa-mujoco-ship" / "include" / "ship" / "environment" / "wave_spectrum.hpp"
WAVE_OUT = PROJECT / "Source" / "HakoniwaShip" / "Public" / "ThirdParty" / "wave_spectrum.hpp"
CPA_SRC = SHIPNAVI / "hakoniwa-mujoco-ship" / "include" / "ship" / "autonomy" / "cpa_ozt.hpp"
CPA_OUT = PROJECT / "Source" / "HakoniwaShip" / "Public" / "ThirdParty" / "cpa_ozt.hpp"
GLB_OUT = PROJECT / "SourceArt" / "Ships"
SHIP_SPECS = SHIPNAVI.parent / "images" / "image2sim-framework" / "output"
# 船載カメラ（SHP-25）：船舶生成器の取り付け位置（mounts の camera_rgb）→ 書き戻す PDU（pdu_config_ship.json）
AUTONOMY_PDU = "autonomy_status"
CAMERA_PDUS = {"camera_bow_center": "camera_bow", "camera_bridge_front": "camera_bridge"}
CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RATE_HZ = 640, 480, 10.0
# 取り付け位置は船舶生成器のカメラの筐体（上部構造物のメッシュの暗い箱、前へ最大 0.58 m）の中。筐体を写さないよう手前を切る [m]
CAMERA_NEAR_CLIP_M = 0.7
# 取り付け位置が船体・上部構造物の閉じた箱の中にあるとき（749GT の camera_bridge_front は操舵室の箱の中）、レンズを前方の面の外へ出す距離 [m]
CAMERA_LENS_OUTSIDE_M = 0.3
PLANT_EXPECTED = GODOT / "tests" / "integration" / "plant_coupling" / "expected.json"
# 物理プラグイン（ship_hydro_plugin.cpp の update_wave_engine_if_needed）と同じ成分数・有効化の閾値
WAVE_COMPONENTS = 32
WAVE_ACTIVE_HS = 0.01
ASSET_NAME = "UnrealShipViewer"

# ship_sync.profile.json の軸 → FLU の単位ベクトル
AXIS_FLU = {"X": [1.0, 0.0, 0.0], "X_FLU_Y_BLENDER": [1.0, 0.0, 0.0], "Y": [0.0, 1.0, 0.0], "Z": [0.0, 0.0, 1.0]}
# 関節ノードの接頭辞（image2sim の build_ship.py）
JOINT_PREFIXES = ("rudder_", "propeller_", "radar_mast_")
# 仮の形の色（sRGB 0..1）。ノード名の接頭辞で決める
PART_COLORS = [
    ("hull_appendages", [0.35, 0.12, 0.10]),
    ("hull", [0.55, 0.13, 0.12]),
    ("superstructure", [0.92, 0.92, 0.90]),
    ("cargo", [0.20, 0.38, 0.62]),
    ("deck_fittings", [0.45, 0.45, 0.45]),
    ("rudder_", [0.30, 0.10, 0.08]),
    ("propeller_", [0.78, 0.62, 0.30]),
    ("radar_mast_", [0.85, 0.85, 0.85]),
    ("navlight_", [1.00, 0.95, 0.60]),
]


def load_json(path: Path):
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def load_glb(path: Path) -> dict:
    b = path.read_bytes()
    magic, _, _ = struct.unpack_from("<III", b, 0)
    if magic != 0x46546C67:
        raise SystemExit(f"not a glb: {path}")
    n, kind = struct.unpack_from("<II", b, 12)
    if kind != 0x4E4F534A:
        raise SystemExit(f"first chunk is not JSON: {path}")
    return json.loads(b[20:20 + n])


def gltf_to_flu(v):
    return [-v[2], -v[0], v[1]]


def r6(v):
    return [round(float(x), 6) + 0.0 for x in v]   # -0.0 を 0.0 に


def mesh_bounds_gltf(g: dict, mesh_index: int):
    """メッシュの全プリミティブの POSITION の外形（glTF のノード座標）。"""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for prim in g["meshes"][mesh_index]["primitives"]:
        acc = g["accessors"][prim["attributes"]["POSITION"]]
        for i in range(3):
            lo[i] = min(lo[i], acc["min"][i])
            hi[i] = max(hi[i], acc["max"][i])
    return lo, hi


def box_flu(lo, hi, offset_gltf):
    """glTF の外形の箱（ノードの平行移動を足す）を FLU の箱に。軸の入れ替えで min/max が入れ替わる。"""
    a = gltf_to_flu([lo[i] + offset_gltf[i] for i in range(3)])
    b = gltf_to_flu([hi[i] + offset_gltf[i] for i in range(3)])
    return [min(a[i], b[i]) for i in range(3)], [max(a[i], b[i]) for i in range(3)]


def part_color(name: str):
    for prefix, color in PART_COLORS:
        if name.startswith(prefix):
            return color
    return [0.6, 0.6, 0.6]


def materials(g: dict) -> dict:
    """glb の材質（glTF の色は線形）。発光は emissiveFactor × KHR_materials_emissive_strength（Godot の glTF 取り込みと同じ）"""
    out = {}
    for m in g.get("materials", []):
        pbr = m.get("pbrMetallicRoughness", {})
        strength = m.get("extensions", {}).get("KHR_materials_emissive_strength", {}).get("emissiveStrength", 1.0)
        out[m["name"]] = {
            "base_color": r6(pbr.get("baseColorFactor", [1.0, 1.0, 1.0, 1.0])[:3]),
            "metallic": round(float(pbr.get("metallicFactor", 1.0)), 6),
            "roughness": round(float(pbr.get("roughnessFactor", 1.0)), 6),
            "emissive": r6([c * strength for c in m.get("emissiveFactor", [0.0, 0.0, 0.0])]),
        }
    return out


def find_key(o, key):
    if isinstance(o, dict):
        if key in o:
            return o[key]
        for v in o.values():
            r = find_key(v, key)
            if r is not None:
                return r
    elif isinstance(o, list):
        for v in o:
            r = find_key(v, key)
            if r is not None:
                return r
    return None


def nav_light_intensity_cd(range_nm: float) -> float:
    """COLREG 附属書 I 第 8 項：I = 3.43e6 × T × D² × K^-D（T = 2e-7 lx、K = 0.8）[cd]"""
    return 3.43e6 * 2.0e-7 * range_nm ** 2 * 0.8 ** (-range_nm)


def nav_lights(model: str, nodes: set) -> list:
    """航海灯：射光範囲の中心は船首から右回り [deg]。マスト灯 0（225°）、右舷灯 +56.25・左舷灯 −56.25（各 112.5°、正船首から正横後 22.5° まで）、
    船尾灯 180（135°）、全周灯 0（360°）。視点の方位がこの範囲にあるときだけ灯火が見える"""
    spec = load_json(SHIP_SPECS / model / "resolved_config.json")
    out = []
    for lt in find_key(spec, "nav_lights") or []:
        node = "navlight_" + lt["id"]
        if node not in nodes:
            raise SystemExit(f"{model}: nav light {node} not in the glb")
        lid = lt["id"]
        arc = float(lt.get("arc_deg", 225.0))
        if arc >= 360.0:
            center = 0.0
        elif "starboard" in lid:
            center = 56.25
        elif "port" in lid:
            center = -56.25
        elif "stern" in lid:
            center = 180.0
        else:
            center = 0.0
        rng = float(lt.get("range_nm", 2.0))
        out.append({"node": node, "color": lt.get("color", "white"), "arc_deg": arc, "center_deg": center,
                    "range_nm": rng, "intensity_cd": round(nav_light_intensity_cd(rng), 3)})
    return out


def build_ship(ship: dict) -> dict:
    model = ship["model"]
    g = load_glb(GODOT / "Models" / model / f"{model}.glb")
    profile = load_json(GODOT / "Models" / model / "ship_sync.profile.json")
    roots = g["scenes"][g.get("scene", 0)]["nodes"]
    if len(roots) != 1 or g["nodes"][roots[0]].get("name") != model:
        raise SystemExit(f"{model}: expected a single root node named {model}")
    root = g["nodes"][roots[0]]
    if any(k in root for k in ("translation", "rotation", "scale", "matrix")):
        raise SystemExit(f"{model}: root node has a transform (not supported)")
    joint_nodes = {j["node_name"]: j for j in profile["joints"]}
    parts, joints = [], []
    mounts = {}
    lo_all = [float("inf")] * 3
    hi_all = [float("-inf")] * 3
    for idx in root.get("children", []):
        nd = g["nodes"][idx]
        name = nd.get("name", "")
        if nd.get("children") or "rotation" in nd or "scale" in nd or "matrix" in nd:
            raise SystemExit(f"{model}/{name}: nested or rotated nodes are not supported")
        t = nd.get("translation", [0.0, 0.0, 0.0])
        if "mesh" not in nd:
            if name.startswith("mount_"):
                mounts[name[len("mount_"):]] = r6(gltf_to_flu(t))
            continue
        lo, hi = mesh_bounds_gltf(g, nd["mesh"])
        is_joint = name.startswith(JOINT_PREFIXES)
        if is_joint and name not in joint_nodes:
            raise SystemExit(f"{model}/{name}: joint node not in ship_sync.profile.json")
        # 関節の部品は関節の原点からの相対、それ以外は船体の座標
        bmin, bmax = box_flu(lo, hi, [0.0, 0.0, 0.0] if is_joint else t)
        part = {"node": name, "box_min": r6(bmin), "box_max": r6(bmax), "color": part_color(name)}
        if not is_joint:
            part["origin"] = r6(gltf_to_flu(t))
        if is_joint:
            j = joint_nodes[name]
            if j["axis"] not in AXIS_FLU:
                raise SystemExit(f"{model}/{name}: unknown axis {j['axis']}")
            joints.append({"name": j["name"], "node": name, "type": j["type"],
                           "axis": AXIS_FLU[j["axis"]], "origin": r6(gltf_to_flu(t)), "part": part})
        else:
            parts.append(part)
        wmin, wmax = box_flu(lo, hi, t)
        for i in range(3):
            lo_all[i] = min(lo_all[i], wmin[i])
            hi_all[i] = max(hi_all[i], wmax[i])
    missing = sorted(set(joint_nodes) - {j["node"] for j in joints})
    if missing:
        raise SystemExit(f"{model}: profile joints without glb nodes: {missing}")
    p = ship.get("preview_pose", {})
    lin, ang = p.get("linear", {}), p.get("angular", {})
    out = {
        "robot": ship["robot"],
        "model": model,
        "pos_pdu": ship["pos_pdu"],
        "preview_pose": {"linear": [lin.get("x", 0.0), lin.get("y", 0.0), lin.get("z", 0.0)],
                         "angular": [ang.get("x", 0.0), ang.get("y", 0.0), ang.get("z", 0.0)]},
        "extent": {"min": r6(lo_all), "max": r6(hi_all)},
        "parts": parts,
        "joints": joints,
        "materials": materials(g),
        "nav_lights": nav_lights(model, {p["node"] for p in parts}),
        "mounts": mounts,
        "ais": ship.get("ais", {}),
    }
    if ship.get("joint_states_pdu"):
        out["joint_states_pdu"] = ship["joint_states_pdu"]
    return out


def build() -> dict:
    fleet = load_json(GODOT / "config" / "fleet.json")
    pdu_config = load_json(SHIPNAVI / "pdu_config_ship.json")
    table = {}
    for robot in pdu_config["robots"]:
        for p in robot.get("shm_pdu_readers", []) + robot.get("shm_pdu_writers", []):
            table[(robot["name"], p["org_name"])] = p
    ships = [build_ship(s) for s in fleet["ships"]]
    wanted = []
    for s in ships:
        wanted.append((s["robot"], s["pos_pdu"]))
        if "joint_states_pdu" in s:
            wanted.append((s["robot"], s["joint_states_pdu"]))
    clock = fleet["physics_clock"]
    if (clock["robot"], clock["pdu"]) not in wanted:
        wanted.append((clock["robot"], clock["pdu"]))
    dist = fleet["disturbance"]
    wanted.append((dist["robot"], dist["pdu"]))
    # 自動化レベル（SHP-25 Step 2 の FOC 画面）：受け取ったかだけを出す（中身の形式は未定、K11）
    own = fleet["overlays"]["own_ship"]
    wanted.append((own, AUTONOMY_PDU))
    cams = cameras(fleet, ships)
    pdus = []
    for robot, name in wanted:
        p = table.get((robot, name))
        if p is None:
            raise SystemExit(f"PDU not in pdu_config_ship.json: {robot}/{name}")
        pdus.append({"robot": robot, "pdu": name, "type": p["type"],
                     "channel_id": p["channel_id"], "pdu_size": p["pdu_size"]})
    for c in cams:
        p = table.get((c["robot"], c["pdu"]))
        if p is None or p["type"] != "sensor_msgs/Image":
            raise SystemExit(f"camera PDU not in pdu_config_ship.json as sensor_msgs/Image: {c['robot']}/{c['pdu']}")
        pdus.append({"robot": c["robot"], "pdu": c["pdu"], "type": p["type"],
                     "channel_id": p["channel_id"], "pdu_size": p["pdu_size"], "write": True})
    return {
        "version": 1,
        "generated_by": "tools/gen_fleet_config.py (from hakoniwa-godot-ship/config/fleet.json, pdu_config_ship.json, Models/*.glb)",
        "asset_name": ASSET_NAME,
        "physics_clock": {"robot": clock["robot"], "pdu": clock["pdu"]},
        "wave": {
            "disturbance": {"robot": dist["robot"], "pdu": dist["pdu"]},
            "num_components": WAVE_COMPONENTS,
            "active_hs_m": WAVE_ACTIVE_HS,
            "preview": fleet["preview_wave"],
        },
        "plant_check": plant_check(),
        # ブリッジ視界・FOC の表示（SHP-25 Step 2）：Godot の工学オーバレイと同じ値（速度の推定の窓・OZT・ベクトルの長さ・航跡）
        "overlays": {k: fleet["overlays"][k] for k in ("own_ship", "velocity_window_s", "trail", "ais", "ozt")},
        "autonomy_status": {"robot": fleet["overlays"]["own_ship"], "pdu": AUTONOMY_PDU},
        "cameras": cams,
        "pdus": pdus,
        "ships": ships,
    }


def glb_triangles_flu(path: Path) -> np.ndarray:
    """glb の全メッシュの三角形（FLU、船体の座標）。形は (n, 3, 3)"""
    b = path.read_bytes()
    n = struct.unpack_from("<I", b, 12)[0]
    g = json.loads(b[20:20 + n])
    bin_at = 20 + n + 8

    def acc(i):
        a = g["accessors"][i]
        v = g["bufferViews"][a["bufferView"]]
        off = bin_at + v.get("byteOffset", 0) + a.get("byteOffset", 0)
        dt = {5126: np.float32, 5123: np.uint16, 5125: np.uint32}[a["componentType"]]
        c = {"VEC3": 3, "SCALAR": 1}[a["type"]]
        return np.frombuffer(b, dtype=dt, count=a["count"] * c, offset=off).reshape(-1, c)

    out = []
    root = g["nodes"][g["scenes"][g.get("scene", 0)]["nodes"][0]]
    for i in root.get("children", []):
        nd = g["nodes"][i]
        if "mesh" not in nd:
            continue
        tr = np.array(nd.get("translation", [0.0, 0.0, 0.0]))
        for prim in g["meshes"][nd["mesh"]]["primitives"]:
            pos = acc(prim["attributes"]["POSITION"]).astype(np.float64) + tr
            flu = np.stack([-pos[:, 2], -pos[:, 0], pos[:, 1]], 1)
            out.append(flu[acc(prim["indices"]).reshape(-1, 3)])
    return np.concatenate(out)


def ray_hits(tris: np.ndarray, o: np.ndarray, d: np.ndarray) -> np.ndarray:
    """三角形との交点までの距離（当たらない三角形は除く、昇順）。Möller–Trumbore"""
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    h = np.cross(d, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-12
    f = np.where(ok, 1.0 / np.where(ok, a, 1.0), 0.0)
    s = o - tris[:, 0]
    u = f * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, e1)
    v = f * (q @ d)
    tt = f * np.einsum("ij,ij->i", e2, q)
    hit = ok & (u >= 0) & (u <= 1) & (v >= 0) & (u + v <= 1) & (tt > 0)
    return np.sort(tt[hit])


def camera_lens(model: str, origin: list):
    """レンズの位置：取り付け位置が閉じた箱の中（6 方向すべてに、カメラの筐体（CAMERA_NEAR_CLIP_M 以内）より先の面がある）なら、
    前方（+x）の最初の面の CAMERA_LENS_OUTSIDE_M 外へ出す。外にあればそのまま。返り値は (位置, 移した距離)"""
    tris = glb_triangles_flu(GODOT / "Models" / model / f"{model}.glb")
    o = np.array(origin, dtype=np.float64)
    beyond = {}
    for name, d in (("+x", (1, 0, 0)), ("-x", (-1, 0, 0)), ("+y", (0, 1, 0)), ("-y", (0, -1, 0)), ("+z", (0, 0, 1)), ("-z", (0, 0, -1))):
        h = ray_hits(tris, o, np.array(d, dtype=np.float64))
        h = h[h > CAMERA_NEAR_CLIP_M]
        beyond[name] = float(h[0]) if len(h) else None
    if all(v is not None for v in beyond.values()):
        dx = beyond["+x"] + CAMERA_LENS_OUTSIDE_M
        return r6([o[0] + dx, o[1], o[2]]), round(dx, 3)
    return r6(o), 0.0


def cameras(fleet: dict, ships: list) -> list:
    """自船の船載カメラ（SHP-25）。位置は glb の mount_<id>（閉じた箱の中なら前の面の外へ出したレンズの位置 origin、元は mount）、
    画角は船舶生成器の mounts。向きは船首方向・水平（船体に固定）"""
    own = fleet["overlays"]["own_ship"]
    ship = next(s for s in ships if s["robot"] == own)
    spec = load_json(SHIP_SPECS / ship["model"] / "resolved_config.json")
    out = []
    for m in find_key(spec, "mounts") or []:
        if m.get("type") != "camera_rgb":
            continue
        mid = m["id"]
        if mid not in CAMERA_PDUS:
            raise SystemExit(f"{ship['model']}: camera mount {mid} has no PDU (CAMERA_PDUS)")
        if mid not in ship["mounts"]:
            raise SystemExit(f"{ship['model']}: mount_{mid} not in the glb")
        lens, moved = camera_lens(ship["model"], ship["mounts"][mid])
        out.append({"robot": own, "pdu": CAMERA_PDUS[mid], "frame_id": mid, "mount": ship["mounts"][mid], "origin": lens,
                    "moved_out_m": moved,
                    "fov_deg": float(m.get("fov_deg", 90.0)), "width": CAMERA_WIDTH, "height": CAMERA_HEIGHT,
                    "rate_hz": CAMERA_RATE_HZ, "encoding": "rgb8", "near_clip_m": CAMERA_NEAR_CLIP_M})
    return out


def plant_check() -> dict:
    """Godot の結合テストの期待値のうち、海面と船体の検証に使うもの（同じ値で比べる）。"""
    e = load_json(PLANT_EXPECTED)
    return {
        "settle_s": e["settle_s"],
        "heave_corr_min": e["heave_corr_min"],
        "waterline_rms_max_m": e["waterline_rms_max_m"],
        "wave": e["wave"],
        "ships": {k: {"lpp_m": v["lpp_m"], "beam_m": v["beam_m"]} for k, v in e["ships"].items()},
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="生成物が最新か確かめるだけ（書かない）")
    a = ap.parse_args()
    text = json.dumps(build(), ensure_ascii=False, indent=2) + "\n"
    wave = WAVE_SRC.read_bytes()
    cpa = CPA_SRC.read_bytes()
    models = sorted({s["model"] for s in load_json(GODOT / "config" / "fleet.json")["ships"]})
    glbs = {GLB_OUT / f"{m}.glb": (GODOT / "Models" / m / f"{m}.glb").read_bytes() for m in models}
    if a.check:
        pairs = [(OUT, OUT.exists() and OUT.read_text(encoding="utf-8") == text),
                 (WAVE_OUT, WAVE_OUT.exists() and WAVE_OUT.read_bytes() == wave),
                 (CPA_OUT, CPA_OUT.exists() and CPA_OUT.read_bytes() == cpa)]
        pairs += [(f, f.exists() and f.read_bytes() == b) for f, b in glbs.items()]
        stale = [str(f) for f, ok in pairs if not ok]
        if stale:
            print(f"FLEET_CONFIG_STALE {' '.join(stale)}")
            return 1
        print("FLEET_CONFIG_OK")
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding="utf-8")
    WAVE_OUT.parent.mkdir(parents=True, exist_ok=True)
    WAVE_OUT.write_bytes(wave)
    CPA_OUT.write_bytes(cpa)
    GLB_OUT.mkdir(parents=True, exist_ok=True)
    for f, b in glbs.items():
        f.write_bytes(b)
    print(f"wrote {OUT}, {WAVE_OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
