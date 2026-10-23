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

* ★ 箱庭の Windows インストーラが `HAKO_CORE_*_PATH`・`HAKO_CONFIG_PATH` を全体の環境変数に入れています（古い `%APPDATA%\hakoCore-win`）。
  `setup_build.ps1` は `-CoreSdkRoot` が無ければ環境変数より `shipnavi\build\shakoc-dll` を優先し、`ship_scene.ps1` は実行時の DLL と共有メモリの設定（`shipnavi\runtime\windows-msvc-x64\cpp_core_config.json`、`Z:/mmap_ship`）を上書きします。
  ログに `loaded shakoc.dll: .../shipnavi/build/shakoc-dll/bin/shakoc.dll` と出れば船の DLL です。
* ★ SDK を替えたときは `Intermediate`・`Binaries`（プラグインのものも）を消してから建て直します。UBT は環境変数の変化を検出しません。
* `-game` のログに出る `LogPython: Error ... unreal.ToolsetDefinition` はエンジン同梱の実験的な Toolsets プラグインのもので、このプロジェクトとは関係ありません。

## 場面（Step 1）

| 要素 | 中身 |
|---|---|
| 座標 | PDU の世界（右手系 m・ENU、船体は FLU）→ Unreal = (100x, −100y, 100z)（`ShipFrame.h`。rover の `RoverFrame.h` と同じ） |
| 光・空 | 太陽（平行光・仰角 35°・大気の太陽）、空の大気、空の光（実時間取得）、高さの霧 |
| 海面 | 仮の板（平均海面 z = 0・4 km 四方）。決定論的 Gerstner 波面（D4、海面の時刻は物理時刻＝D19）は Step 3 |
| 見る人 | 姿の無い観客（`AShipViewGameMode`）。マウスは捕まえない |

## これから（SHP-24）

1. Step 2：`AShipActor` に船の姿勢（D14・D18）と関節（`joint_states`）を反映する
2. Step 3：海面を D4 の Gerstner 波で描く（時刻は D19）
3. Step 4：新しい glb（塗装・上部構造物つき）を取り込み、Godot 側と見た目をそろえる

## ライセンス

MIT License（`LICENSE`）。
