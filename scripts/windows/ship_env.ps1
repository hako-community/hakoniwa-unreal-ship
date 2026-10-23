# ship_env.ps1 — ship_scene.ps1・run_plant_test.ps1 が読み込む共通の設定（. で読む）。
#
# 実行時の shakoc.dll と共有メモリの設定は船のもの（shipnavi\build\shakoc-dll・shipnavi\runtime の Z:/mmap_ship）。
# ★ 箱庭の Windows インストーラが HAKO_CORE_DLL_PATH・HAKO_CONFIG_PATH を全体の環境変数に入れている（古い SDK）ので、ここで上書きする
param(
    [string]$EngineRoot = (Join-Path $env:ProgramFiles 'Epic Games\UE_5.8')
)

$ShipRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$ShipNavi = (Resolve-Path (Join-Path $ShipRoot '..')).Path
# 船舶シミュレータ共通の環境（SDK の PATH・HAKO_BINARY_PATH、共有メモリのフォルダ Z:/mmap_ship を作る）。
# hakoniwa-godot-ship の godot_env.ps1 と同じく shipnavi\scripts\env_ship.ps1 を読む（無いと conductor が hako-master を作れない）
$ShipEnvCommon = Join-Path $ShipNavi 'scripts\env_ship.ps1'
if (-not (Test-Path $ShipEnvCommon)) { throw "見つかりません: $ShipEnvCommon" }
. $ShipEnvCommon | Out-Null
$ShipUproject = Join-Path $ShipRoot 'HakoniwaShip.uproject'
$ShipEditor = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
$ShipMap = '/Game/Ship/Levels/ShipSea'
$env:HAKO_CORE_DLL_PATH = Join-Path $ShipNavi 'build\shakoc-dll\bin'
$env:HAKO_CONFIG_PATH = Join-Path $ShipNavi 'runtime\windows-msvc-x64\cpp_core_config.json'
foreach ($f in @($ShipEditor, (Join-Path $env:HAKO_CORE_DLL_PATH 'shakoc.dll'), $env:HAKO_CONFIG_PATH)) {
    if (-not (Test-Path $f)) { throw "見つかりません: $f" }
}
$ShipLogDir = Join-Path $ShipRoot 'Saved\ShipLogs'
New-Item -ItemType Directory -Force $ShipLogDir | Out-Null
