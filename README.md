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
pwsh scripts\windows\run_plant_test.ps1 -CheckSec 60 -Screenshot   # 終わりに自船を追うカメラで撮る（Saved\ShipLogs\plant_<時刻>\view.png）
```

* 船の一覧・PDU・形は Godot ビューアと同じ（`tools/gen_fleet_config.py` が `hakoniwa-godot-ship/config/fleet.json`・`pdu_config_ship.json`・`Models/*.glb`・`ship_sync.profile.json` から作る。手で写さない）。
  関節の原点・回転軸は glb のノードとプロファイルから。Step 2 の形はメッシュごとの外形の箱（仮。Step 4 で glb に置き換える）。
* `AShipFleet`（地図に 1 つ）が船ごとに `AShipActor` を置く。`-ShipSync` で `AHakoniwaShmClient`（アセット名 `UnrealShipViewer`）を置いて登録し、`-ShipHakoStart` で開始を要求する。
  姿勢は D14・D18（`R = Rz(yaw)·Ry(−ang_y)·Rx(roll)`、Unreal へは S·R·S）、関節は `joint_states`、物理時刻は OwnShip の `joint_states` の stamp（D19）。
* 検証（`run_plant_test.ps1`）：
  1. Unreal の中（`HAKONIWA_UNREAL_SHIP_SYNC_OK`）：受信数、アクタの位置、エンジンが求めた前・右・上の向き、関節の原点の世界の位置（p + R·origin）、関節の部品の角度を、PDU から右手系で直接求めた値と比べる（位置 1e-3 m・向き 1e-4・関節 1e-4 rad 以内）。
  2. 物理の trace との照合（`tools/compare_plant_trace.py`、`HAKONIWA_UNREAL_SHIP_TRACE_OK`）：アクタから読み戻した姿勢を、同じ物理時刻の `ship_plant --trace` と比べる。
  3. Unreal に読み込まれた shakoc.dll が船のもの 1 つだけか。
* ★ `PduManager::ReadPduRawData` は受信した PDU を 1 回だけ返す（読むと消える）。新しい値が無いフレームは前の姿勢のまま。
* 状態のログ（5 秒ごと）に fps・ゲーム／描画スレッドと GPU の時間・最長フレーム・前面の割合を出す。

## 場面（Step 1）

| 要素 | 中身 |
|---|---|
| 座標 | PDU の世界（右手系 m・ENU、船体は FLU）→ Unreal = (100x, −100y, 100z)（`ShipFrame.h`。rover の `RoverFrame.h` と同じ） |
| 光・空 | 太陽（平行光・仰角 35°・大気の太陽）、空の大気、空の光（実時間取得）、高さの霧 |
| 海面 | 仮の板（平均海面 z = 0・4 km 四方）。決定論的 Gerstner 波面（D4、海面の時刻は物理時刻＝D19）は Step 3 |
| 見る人 | 姿の無い観客（`AShipViewGameMode`）。マウスは捕まえない |

## これから（SHP-24）

1. ~~Step 2：`AShipActor` に船の姿勢（D14・D18）と関節（`joint_states`）を反映する~~（済）
2. Step 3：海面を D4 の Gerstner 波で描く（時刻は D19）
3. Step 4：新しい glb（塗装・上部構造物つき）を取り込み、Godot 側と見た目をそろえる

## ライセンス

MIT License（`LICENSE`）。
