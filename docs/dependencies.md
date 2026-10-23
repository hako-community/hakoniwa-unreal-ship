# External Dependencies

このプロジェクトは、公開リポジトリに含めないネイティブバイナリ（箱庭コアの `shakoc`）を前提にしています。hakoniwa-unreal-rover の `docs/dependencies.md` と同じ仕組みで、既定の場所だけが船用です。

## 公開リポジトリに含めないもの

- `Binaries/Win64/shakoc.dll`
- `Plugins/HakoniwaPdu/Source/ThirdParty/shakoc`
- 場面の生成物 `Content/Ship/Levels/`（`scripts/windows/ship_scene.ps1 -Build` が作る）

## shakoc の解決順序

| 段階 | 場所 |
|---|---|
| ビルド（`setup_build.ps1`） | `-CoreSdkRoot` → `shipnavi\build\shakoc-dll`（`shipnavi\scripts\build_shakoc_dll.cmd` が作る DLL 版）。どちらかから `HAKO_CORE_INC_PATH` / `HAKO_CORE_LIB_PATH` / `HAKO_CORE_DLL_PATH` をプロセスに設定し、`HakoniwaPdu.Build.cs` が読む |
| 実行（`HakoniwaPdu` の DLL ローダ） | `HAKO_CORE_DLL_PATH` → ビルド時の SDK の DLL → `%APPDATA%\hakoCore-win\bin` → リポジトリ内。`ship_scene.ps1` は `HAKO_CORE_DLL_PATH` を船の DLL に、`HAKO_CONFIG_PATH` を `shipnavi\runtime\windows-msvc-x64\cpp_core_config.json`（`Z:/mmap_ship`）に上書きする |

`build_shakoc_dll.cmd` は共有 SDK（`shipnavi\sdk\windows-msvc-x64`、`devai_robo` へのジャンクション）の箱庭コアのソースを `-DHAKO_WIN32_SHARED_LIBS=ON` で建て、出力は `shipnavi\build\` に置きます（共有 SDK には書かない）。船の PLANT と同じ版のコアなので、共有メモリの形式が一致します。

## 現時点の制約

Win64 だけを対象にしています。
