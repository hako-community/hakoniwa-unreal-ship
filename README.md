# Hakoniwa Unreal Ship

自動運航船シミュレータ（MEGURI2040 / DFFAS+）の主ビューア。Unreal Engine 5.8 で、箱庭の船舶 PLANT（[hakoniwa-mujoco-ship](https://github.com/hako-community/hakoniwa-mujoco-ship) の `ship_plant`）の状態を描きます。工学・高速検証用は [hakoniwa-godot-ship](https://github.com/hako-community/hakoniwa-godot-ship) です。

* [hakoniwa-unreal-rover](https://github.com/hako-community/hakoniwa-unreal-rover)（hakoniwa-unreal-drone をコピーしてドローン部分を外したもの）をもとに作りました（2026-10-23、SHP-24 Step 1）。
  箱庭の PDU 通信（`Plugins/HakoniwaPdu`）と、箱庭のアセット登録（`AHakoniwaShmClient`）はそのまま使います。
* 物理は hakoniwa-mujoco-ship が受け持ちます。このプロジェクトは描画だけです。

## セットアップ

1. Unreal Engine 5.8 をインストールします。
2. サブモジュールを取得します：`git submodule update --init --recursive`
3. 箱庭コアの shakoc を **DLL で**建てます（船の作業ルート `shipnavi` で）。共有 SDK の shakoc は静的版だけで、Unreal には DLL が要ります。

```powershell
cmd /c ..\scripts\build_shakoc_dll.cmd       # → shipnavi\build\shakoc-dll（bin・lib・include）
pwsh scripts\windows\setup_build.ps1          # Editor ビルド（既定で上の DLL 版を使う。Windows PowerShell 5.1 でも可）
pwsh scripts\windows\ship_scene.ps1 -Build    # /Game/Ship/Levels/ShipSea を作る（★ clone の後に必須。エディタが自動で開いて閉じる）
pwsh scripts\windows\ship_scene.ps1 -Smoke    # -game で数秒描き、HAKONIWA_UNREAL_SHIP_BOOT_OK とフレーム数・fps を出す
pwsh scripts\windows\ship_scene.ps1 -Open     # エディタで開く
```

* ★ 箱庭の Windows インストーラが `HAKO_CORE_*_PATH`・`HAKO_CONFIG_PATH` と PATH（`%APPDATA%\hakoCore-win\bin`）を全体の環境変数に入れています（古い SDK）。
  `setup_build.ps1` は `-CoreSdkRoot` が無ければ環境変数より `shipnavi\build\shakoc-dll` を優先します。
  `ship_scene.ps1`・`run_plant_test.ps1` は `scripts\windows\ship_env.ps1` で `shipnavi\scripts\env_ship.ps1`（共有メモリのフォルダ `Z:/mmap_ship` を作る）を読み、実行時の DLL と共有メモリの設定を船のものにします。
  `HakoniwaPdu` は shakoc.dll を**遅延読み込み**にしています（直接の import だと Windows が PATH から古い DLL を読む）。`run_plant_test.ps1` は Unreal に読み込まれた shakoc.dll が船のもの 1 つだけかを確かめます。
* ★ SDK を替えたときは `Intermediate`・`Binaries`（プラグインのものも）を消してから建て直します。UBT は環境変数の変化を検出しません。
* `-game` のログに出る `LogPython: Error ... unreal.ToolsetDefinition` はエンジン同梱の実験的な Toolsets プラグインのもので、このプロジェクトとは関係ありません。

## 船団と実物理との結合（Step 2）

```powershell
py -3.12 tools\gen_fleet_config.py              # Config\Fleet\fleet_ue.json を作り直す（--check で最新か確かめる）
pwsh scripts\windows\run_plant_test.ps1         # conductor → ship_plant ＋ 外乱 → Unreal。成功で HAKONIWA_UNREAL_SHIP_PLANT_OK
pwsh scripts\windows\run_plant_test.ps1 -CheckSec 60 -Screenshot   # 終わりに 3 画面を撮る（Saved\ShipLogs\plant_<時刻>\view_chase.png・view_bridge.png・view_foc.png）
```

* 船の一覧・PDU・形は Godot ビューアと同じ（`tools/gen_fleet_config.py` が `hakoniwa-godot-ship/config/fleet.json`・`pdu_config_ship.json`・`Models/*.glb`・`ship_sync.profile.json` から作る。手で写さない）。
  関節の原点・回転軸は glb のノードとプロファイルから。形は Step 4 で glb のメッシュに（下）。
* `AShipFleet`（地図に 1 つ）が船ごとに `AShipActor` を置く。`-ShipSync` で `AHakoniwaShmClient`（アセット名 `UnrealShipViewer`）を置いて登録し、`-ShipHakoStart` で開始を要求する。
  姿勢は D14・D18（`R = Rz(yaw)·Ry(−ang_y)·Rx(roll)`、Unreal へは S·R·S）、関節は `joint_states`、物理時刻は OwnShip の `joint_states` の stamp（D19）。
* 検証（`run_plant_test.ps1`）：
  1. Unreal の中（`HAKONIWA_UNREAL_SHIP_SYNC_OK`）：受信数、アクタの位置、エンジンが求めた前・右・上の向き、関節の原点の世界の位置（p + R·origin）、関節の部品の角度を、PDU から右手系で直接求めた値と比べる（位置 1e-3 m・向き 1e-4・関節 1e-4 rad 以内）。
  2. 物理の trace との照合（`tools/compare_plant_trace.py`、`HAKONIWA_UNREAL_SHIP_TRACE_OK`）：アクタから読み戻した姿勢を、同じ物理時刻の `ship_plant --trace` と比べる。
  3. Unreal に読み込まれた shakoc.dll が船のもの 1 つだけか。
* ★ `PduManager::ReadPduRawData` は受信した PDU を 1 回だけ返す（読むと消える）。新しい値が無いフレームは前の姿勢のまま。
* 状態のログ（5 秒ごと）に fps・ゲーム／描画スレッドと GPU の時間・最長フレーム・前面の割合を出す。

## 海面・空（Step 3）

```powershell
pwsh scripts\windows\ship_scene.ps1 -Build                 # 材質（/Game/Ship/Generated）と地図を作り直す
pwsh scripts\windows\ship_scene.ps1 -Smoke -Shot           # 波面の計算の一致・描いた海面の高さを確かめ、画面を撮る
pwsh scripts\windows\run_plant_test.ps1 -CheckSec 60 -Screenshot   # 実物理との結合（海面の検証を含む）
```

* **波面は物理と同じコード**（D4）：hakoniwa-mujoco-ship の `wave_spectrum.hpp` を `Source/HakoniwaShip/Public/ThirdParty/` に写して使う
  （`tools/gen_fleet_config.py` が写し、`--check` で同じか確かめる）。JONSWAP 32 成分の Gerstner 波。成分の表（波数・振幅・位相）を
  CPU が倍精度で作り、位相に「メッシュの中心と時刻」を畳み込んで浮動小数のテクスチャで GPU に渡す（Godot の `ocean.gd` と同じ考え方）。
  頂点は上下だけ動かすので、描いた海面は物理の η(x, y, t) と同じ点で同じ高さ。式は `ShipOceanShader.h`（材質の HLSL）。
* **波浪パラメータ**は Disturbance PDU（`OwnShip/maritime_disturbance`、Godot と同じ取り出し）。箱庭につながないときは `fleet_ue.json` の `wave.preview`。
* **表示時刻**：物理は 50 Hz、描画は 130〜200 FPS。受信した値をそのまま描くと 2〜4 フレームごとに飛んでカクつくので、
  なめらかに進む表示時刻（最新の物理時刻の 0.04 s 後ろ）で船・関節を補間し、海面も同じ時刻で描く（D19 の考え方のまま、船と海面は同じ時刻）。
  ★ `ship_plant` は全船の `pos` を書いてから時刻つきの `joint_states` を書くので、`pos` を先に読み、時刻が届いたらその時刻の値とする。
* **メッシュ**：2.4 km 四方・4 m 格子（カメラに付いて格子の間隔で動く）＋外側 40 km 四方の平面（法線は同じ式。180 km にすると光が当たらず黒い帯になった）。
  見た目だけの細かい波（40 成分、波長 0.6〜20 m・主波向 ±90° の乱数）は法線にだけ足す。遠くは傾きを弱めて粗さを上げる。
* **帯域制限**（SHP-26 で「海がカクカク」の指摘）：短い波の多い海況（Hs 0.8 m・Tp 5 s、最短波長 6 m）で、4 m 格子の頂点に全成分を入れると
  網目の升目の模様（エイリアス）、画素より細かい波を法線に入れると遠くで菱形の模様（モアレ）になった。
  * 頂点：波長が格子の 3〜5 倍で 0→1 の重み（`VertexWeight`）。除いた短い波（Hs 0.8 m で最大 0.22 m、Hs 2.5 m で 0.30 m）は法線だけで描く。
    描いた海面の検証は同じ重みの高さ（`MeshElevationAt`）と比べ（誤差 9e-5 m）、除いた分はログに別に出す（`short waves left to normals`）。
  * 画素：画素の大きさ（距離 × 0.0015）の 2〜6 倍より短い波を成分ごとに傾きから外す。
  * 物理の成分は数十本の正弦波なので、細かい波が少なく規則的だと升目の干渉模様が見える → 細かい波を不規則に（上）。
* **開始の前の待ち**：`-ShipHakoStart` は 25 ms 未満のフレームが 120 続いてから開始を要求する。起動直後の重いフレームの間に始めると
  物理が先に進み、表示時刻が 2 倍速で追いかけて海の動きが不自然になった（デモで最大 1.8 s 遅れ → 0.3 s）。
* **空**：江の島の Godot シーン（`D:\work_godot\3DMode_enoshima_PC`）と同じ Poly Haven の空パノラマ（CC0）を 2k に縮めたもの
  （`tools/make_sky_hdri.py` → `SourceArt/Sky/sky_2k.exr`・`sky.json`）。空の球に貼って空の光が実時間で取り込む。太陽（平行光）はパノラマの太陽の向き・
  照度（約 9 万 lx）で、パノラマの太陽の円盤は抑えてある。霧は水平線の空の色。水の色は江の島の海の deep / shallow。
* ★ 物理単位の昼光（露出 EV 13 前後）なので `r.EyeAdaptation.CachedLightingPreExposure=8`（`Config/DefaultEngine.ini`）。
  既定の 4 だと Lumen と空の光の取り込みがクリップされ、画面に赤い警告が出る（ログには出ない。`-ShipShot` の画像には写る）。
* 検証：
  1. `-ShipWaveParity=<reference.json>`：`wave_spectrum.hpp` の成分と波面を Godot の `tests/unit/wave_parity/reference.json`（Python 実装の値）と比べる（`HAKONIWA_UNREAL_SHIP_WAVE_PARITY_OK`）。
  2. 描いた海面の高さ（`AShipOcean`）：真上からの正射影の深度で、描いた海面を格子の頂点で読み、CPU の η と比べる（許容 5 mm。実測 9e-5 m）。
  3. 結合テスト：波浪パラメータ、海面の時刻 = 船の表示時刻、船体の上下動と海面の相関（Godot と同じ閾値 0.8・水線 RMS 0.4 m）、
     表示時刻が止まったフレームの割合（5 % 以下）。物理の trace は表示時刻に同じ式で補間して照合する。

## 船の形・露出・なめらかさ（Step 4）

* **船の形**：Godot ビューアと同じ glb（`tools/gen_fleet_config.py` が `SourceArt/Ships/<model>.glb` に写し、`--check` で同じか確かめる）を、
  場面づくり（`ship_scene.ps1 -Build`）が部品（メッシュのノード）ごとの StaticMesh（`/Game/Ship/Generated/Ships/<model>/SM_<node>`、Nanite）にする
  （`Source/HakoniwaShipEditor/Private/ShipModelImport.cpp`）。エンジンの glTF 取り込みは使わず、座標は自分で移す：glTF → FLU（x = −Z, y = −X, z = Y、
  `gen_fleet_config.py` と同じ）→ Unreal。三角形の表の向きが glb の法線と合うかを数える（4 隻 53,291 三角形で逆向き 0）。
* `AShipActor` は部品の原点に StaticMesh を置き、材質の差し込み口（glb の材質名）に `fleet_ue.json` の `materials`（色・金属度・粗さ・発光、
  Godot と同じ値）を入れる（`M_ShipPaint`、両面）。メッシュの外形を `fleet_ue.json` の外形の箱と比べる（`SHIP_MODEL ... max bounds err`、実測 2e-6 m）。
  メッシュが無いとき（場面を作り直していない）は外形の箱を仮の形として置き、検証は失敗する。
* **露出**：晴天の昼に固定（EV100 15.5、場面の `DaylightExposure`）。自動露出は暗い海に合わせて明るくしすぎ、紺の船体が水色に見えた。
  `-ShipExposure=<EV100>` で上書きできる（調整用）。
* **なめらかさ**：結合テストは、表示時刻が止まったフレーム（5 % 以下）、合わせ直し（0 回）、始まって 5 s 後からの 25 ms を超えたフレーム（2 回まで）を確かめる。
  描いた海面の高さの検証は GPU の写しを数フレーム後に受け取る（同期で読むと 10 s ごとに 28 ms 止まった）。
  ★ `ship_plant` は全船の `pos` を書いてから時刻つきの `joint_states` を書く。`pos` に時刻が無いので、`pos` → 時刻 →（時刻が届いたら）もう一度 `pos` の順に読む。
* `-ShipFrameLog=<csv>`：フレームごとの壁時計・物理時刻・表示時刻（カクつきの調査用）。

## 太陽光のまぶしさ・夜・航海灯（Step 5）

```powershell
pwsh scripts\windows\ship_scene.ps1 -Smoke -Shot -TowardLight      # 昼：太陽の方を向いて撮る（太陽の円盤・ブルーム・レンズフレア）
pwsh scripts\windows\ship_scene.ps1 -Smoke -Shot -Night            # 夜：月・星・航海灯。航海灯の射光範囲を確かめる（HAKONIWA_UNREAL_SHIP_NAVLIGHT_OK）
pwsh -Command "& .\scripts\windows\run_plant_test.ps1 -Screenshot -ExtraArgs @('-ShipNight')"   # 夜の結合テスト
```

* `AShipLighting`（場面に 1 つ）が昼と夜を切り替える（`-ShipNight`。場面の時刻は PDU に無いので起動の引数で選ぶ）。
  * 昼：パノラマの太陽（約 9 万 lx）、空に太陽の円盤（視半径 0.265°、ブルームとレンズフレアでまぶしさ）、露出 EV100 15.5。
  * 夜：満月（0.25 lx、青みの光）、空はパノラマを暗く青くしたもの（月夜の空 約 0.03 cd/m²）と星・月の円盤、霧も暗く、露出 EV100 0.5。
  * `-ShipExposure=<EV100>` で露出を上書き（調整用）。
* **航海灯**：灯の色・射光範囲・視認距離は船舶生成器の出力（`../../images/image2sim-framework/output/<model>/resolved_config.json`、物理 `ship_plant` と同じ元）から
  `tools/gen_fleet_config.py` が `fleet_ue.json` の `nav_lights` にする。射光範囲の中心は COLREG 第 21 条（マスト灯 正船首 225°、舷灯 正船首から正横後 22.5° まで 112.5°、
  船尾灯 正船尾 135°、全周灯 360°）、光度は附属書 I の式（6 海里 94 cd、3 海里 12 cd、2 海里 4.3 cd、1 海里 0.86 cd）。
  * 灯ごとに材質を分け、視点の方位が射光範囲の外なら発光しない（`M_ShipPaint` の `SectorCenter` / `SectorHalf`、式は `ShipOceanShader.h`）。灯の輝度 = 光度 / 灯の見かけの面積。
  * 夜は光源も点ける（射光範囲 160° 以下はスポット、ほかは点光源。光度 [cd]、影なし）。
  * 検証（`-ShipNavLightCheck`）：各灯を周り 36 方位から 10 m で撮り、範囲の内（端から 3° 以上内側）は最大の 30 % 以上で色が合い、外は 2 % 以下（4 隻 18 灯・616 方位）。
    近くの灯の光源に照らされた灯の胴を数えないよう、撮る間は光源を消す。左右の取り違え（方位の符号）を入れると FAIL になることを確かめた。
* ★ 動的な材質のインスタンスは動的なインスタンスを親にできない（灯の材質は元の材質から作り、値を入れ直す）。
* 遠くの船（数 km）の灯は画素より小さく、ほとんど見えない（残り）。

## 船載カメラの PDU 書き戻し（SHP-25 Step 1）

```powershell
pwsh scripts\windows\run_plant_test.ps1 -CheckSec 60      # 受け手のアセットが画像 PDU を読み、水平線を姿勢と照合する（CAMERA_PDU_OK / CAMERA_HORIZON_OK）
pwsh scripts\windows\ship_scene.ps1 -Smoke                # （箱庭なし）
# 画像を見る：Unreal に -ShipCameraDump=<dir> を付けると 2 s ごとに PNG を保存する
```

| PDU | ch | 型 | 中身 |
|---|---:|---|---|
| `OwnShip/camera_bow` | 17 | `sensor_msgs/Image` | 船首カメラ（取り付け位置 `camera_bow_center`、水平画角 90°） |
| `OwnShip/camera_bridge` | 18 | `sensor_msgs/Image` | 船橋前面カメラ（`camera_bridge_front`、水平画角 110°） |

* 640×480・`rgb8`（無圧縮、sRGB）・10 Hz。PDU の大きさは 1 MiB（画像 921,600 B ＋ 見出し 312 B が入らなかったので `pdu_config_ship.json` を変更）。
* `UShipCamera`：自船の取り付け位置（船舶生成器の `mounts`、glb の `mount_<id>`）にシーンキャプチャを置き、画面と同じ見た目（ポストプロセス後：
  太陽光のまぶしさ・霧・夜の灯火・露出）を撮る。向きは船首方向・水平（船体に固定、揺れと一緒に動く）。
  * 撮る時刻：表示時刻（D24）が 0.1 s の区切りを越えたフレーム。`header.stamp` はそのフレームの表示時刻（船と海面がその時刻で描かれている）。2 台は半周期ずらす。
  * 読み出しは非同期（GPU の写しを数フレーム後に受け取る。最大 4 フレーム）。描画は止めない（2 台で 113 FPS、25 ms 超 0）。
  * ★ 書き込みの描画先は `RTF_RGBA8_SRGB`（`RTF_RGBA8` だと線形のまま入り、画素の値が 2 段ほど暗い）。
  * ★ 取り付け位置は船舶生成器のカメラの筐体（上部構造物のメッシュ）の中なので、手前 0.7 m を切る（`near_clip_m`）。
  * 取り付け位置は船舶生成器で直した（K10、`image2sim-framework` `302f969`）：船橋カメラは操舵室の前面の 0.15 m 前、船首カメラはブルワークの上端＋1.5 m（支柱つき）。
    以前は船橋カメラが操舵室の閉じた箱の中、船首カメラがブルワークと同じ高さだった。安全策として `gen_fleet_config.py` は、取り付け位置が閉じた箱の中
    （6 方向に光線を飛ばしてすべて面に当たる）ならレンズを前の面の 0.3 m 外へ出す（`fleet_ue.json` の `mount`・`origin`・`moved_out_m`。今は 0）。
* 検証（`run_plant_test.ps1`）：
  1. Unreal の中：撮った数・書いた数（表示時刻の長さ × 10 Hz の 9 割以上）、書き込みの失敗・捨てた画像 0、画像の平均の明るさ。
  2. 受け手（`tools/check_camera_pdu.py`、SDK の Python で箱庭のアセットとして読む）：数、時刻の順と間隔（0.1 s、1.5 周期を超える空き 2 % 以下）、
     形式（frame_id・640×480・rgb8・step・データの長さ）、明るさ → `CAMERA_PDU_OK`。★ 読み終えても時刻を進め続ける（箱庭は全アセットの足並みをそろえる）。
  3. 水平線（`tools/compare_camera_horizon.py`）：船橋カメラの画像の左右の端の列で空と海の境目を海のきめ（粗さ）で見つけ、画像の時刻の姿勢（trace）から求めた
     行と比べる。見つかるのは波面メッシュの外周（真の水平線より約 2.4 画素下）なので、走行全体の一定のずれ（0〜6 画素）を引いた残差で判定（中央値 1・最大 4 画素）
     → `CAMERA_HORIZON_OK`。取り付けを 2° 上に向けると FAIL になることを確かめた。（0.1 s の時刻のずれは、この海況の揺れの速さでは 0.3 画素ほどで見分けられない）
* ★ 箱庭の共有メモリ（`Z:/mmap_ship` の `mmap-*.bin`）は前の実行の大きさのまま残り、PDU の表が大きくなっても広がらない（Unreal が書き込みで落ちた）。
  `run_plant_test.ps1` は前の箱庭が動いていなければ始める前に消す。

## ブリッジ視界・FOC 画面（SHP-25 Step 2）

```powershell
pwsh scripts\windows\run_plant_test.ps1 -CheckSec 60 -Screenshot   # 航海情報を trace と照合（HAKONIWA_UNREAL_SHIP_NAV_OK）、終わりに 3 画面を撮る
# キー 1：自船を追う  2：船橋からの視界  3：FOC  0：自由（観客）。起動の引数 -ShipView=chase|bridge|foc|free
```

* 船橋（キー 2）：目は船橋カメラの取り付け位置（操舵室の前面）の 0.5 m 前、水平画角 75°、船と一緒に揺れる。
  * 左上の表示板：自船の HDG・COG・SOG・ROT・舵角（右舵正。物理の舵角は左舵正）・プロペラ回転数・波（Hs・Tp・進む向き）・水深・他船の数と衝突の危険の数・自動化レベル。
  * 上：船首方位の目盛り（カメラの向き）に HDG（白）と COG（水色）の印。
  * 他船のタグ（画面上の船の位置）：船名（AIS の無い船は黄で `NON-AIS (radar)`）・距離・方位（相対方位）・CPA/TCPA。衝突の危険は赤。
* FOC（キー 3）：画面全体を覆う陸上監視センターの表示板。
  * 左：北が上・自船中心の海図（半径 1.6 NM、0.5 NM ごとの距離環）。各船の向き・6 分の速度ベクトル・1 s ごとの航跡（最大 900 点）、OZT の危険進路の弧（橙）。
  * 右上：船首・船橋カメラの画像（PDU に書いているのと同じ描画先）。右下：航海情報と他船の表（距離・方位・CPA・TCPA・ALARM）。
  * 自動化レベル（`OwnShip/autonomy_status` ch 50、`std_msgs/String` の JSON、D29。SUT `ship_sut` が 5 Hz で書く）：
    `AUTONOMY AL4 ODD 0.62 AVOID_STBD CMD 147.8 10.0 kn` のように AL・ODD 余裕度・航行状態・指令針路/速力を出す（AL4 緑・AL3 黄・AL2 以下赤、
    1 s 以上届かなければ灰色で stale、未受信は「no data」）。デモは `hakoniwa-mujoco-ship/demos/run_ship_demo.ps1`。
* 値の求め方は Godot の工学オーバレイと同じ（`ShipNav.h`）：物理の受信値（表示用に補間する前）の直近 5 s の最小二乗で速度と回頭角速度、位置と船首方位は最後の受信値。
  CPA/TCPA・OZT は `cpa_ozt.hpp`（`hakoniwa-autonomy-ship` から `gen_fleet_config.py` が写す。閾値 926 m / 900 s、OZT の安全離隔 926 m・先読み 900 s）。
* 表記は英語（エンジンの既定の字体に日本語の字形が無い）。
* 検証：`-ShipHudLog=<csv>` で 1 s ごとの表示値を書き、`tools/compare_nav_hud.py` が trace から独立に同じ計算をして比べる（始まりの 6 s は除く：開始直後は物理が
  遅れを取り戻すため数ステップずつ進み、Unreal が受け取れない標本がある）。60 s の結合試験で SOG 0.001 kn・COG 0.006°・ROT 0.08°/min・CPA 0.00003 NM、
  衝突の危険の判定の食い違い 0、25 ms を超えるフレーム 0。
* `-ShipShotViews=chase,bridge,foc`（`-ShipShot` と一緒に）：終わりに画面を順に切り替えて `<名前>_<画面>.png` を撮る。

## 統合デモの表示（SHP-26）

デモは `hakoniwa-mujoco-ship/demos/run_ship_demo.ps1`（物理・外乱・SUT `ship_sut`・FOC の操作員・このビューアを起動し、終わりに自動採点）。

* **霧（デモ③、D30）**：外乱 PDU の視程（`d_user_custom[0][1]`）が 5 NM 未満なら、高さフォグの濃さを 81.4 / 視程 [m] にし、空の球にも霧を掛ける
  （エンジンの霧の式：密度 FogDensity/1000 [1/cm]、水平の透過率 exp(−(ln2)²·FogDensity·1e-3·距離 [cm])、視程はコントラスト 2 %）。
  5 NM 以上は「視界良好」として場面の霞のまま（既定の 10 km で霞ませると船橋カメラの水平線の検証が崩れた）。表示板に `VIS 1.2 NM`。
* **GNSS 異常・FOC の介入（デモ③）**：ch 50 の `gps` が 0 なら `GNSS FAULT (DR)`（推測航法）を赤で、AL2 の `MRS`、AL1 の `REMOTE` を出す。
  FOC 画面の下に最後の FOC の指令（`OwnShip/foc_command` ch 51、`{"cmd":"takeover","crs":90.0,"spd":6.0}` など）。
* **岸壁（デモ②、D31）**：`-ShipQuay=x1,y1,x2,y2,水側の点x,水側の点y`（ENU [m]、物理の `quay` と同じ線分）で、陸側へ幅 25 m・水面上 2 m の
  コンクリート、12 m おきの防舷材（突出 1 m）、20 m おきの係船柱を置く（エンジン同梱の立方体）。

## 場面（Step 1）

| 要素 | 中身 |
|---|---|
| 座標 | PDU の世界（右手系 m・ENU、船体は FLU）→ Unreal = (100x, −100y, 100z)（`ShipFrame.h`。rover の `RoverFrame.h` と同じ） |
| 光・空 | Step 3 で空のパノラマ・パノラマの太陽・霧に（上） |
| 海面 | Step 3 で決定論的 Gerstner 波面に（上） |
| 見る人 | 姿の無い観客（`AShipViewGameMode`）。マウスは捕まえない |

## これから（SHP-24）

1. ~~Step 2：`AShipActor` に船の姿勢（D14・D18）と関節（`joint_states`）を反映する~~（済）
2. ~~Step 3：海面を D4 の Gerstner 波で描く（時刻は D19）~~（済）
3. ~~Step 4：新しい glb（塗装・上部構造物つき）を取り込み、Godot 側と見た目をそろえる~~（済）
4. ~~Step 5：太陽光のまぶしさ・夜・航海灯~~（済）。残り：遠くの灯の見え方（画素より小さい灯）
5. SHP-25：~~Step 1 船載カメラの PDU 書き戻し~~（済）、~~Step 2 ブリッジ視界・FOC 画面~~（済）、~~自動化レベルの形式（K11）~~（済、D29）
6. SHP-26：~~デモ① 輻輳海域・② 横風下の自動着桟・③ 濃霧＋GNSS 異常の表示~~（済）

## ライセンス

MIT License（`LICENSE`）。
