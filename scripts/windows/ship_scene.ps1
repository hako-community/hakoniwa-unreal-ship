# ship_scene.ps1 — 海の場面（SHP-24）を組む・起動を確認する・エディタで開く。hakoniwa-unreal-rover の rover_scene.ps1 と同じ形。
#
#   pwsh scripts\windows\ship_scene.ps1 -Build            # /Game/Ship/Levels/ShipSea を作り直す（clone の後に必須。エディタが自動で開いて閉じる）
#   pwsh scripts\windows\ship_scene.ps1 -Smoke            # -game で開き、数秒描いて HAKONIWA_UNREAL_SHIP_BOOT_OK を確認する
#   pwsh scripts\windows\ship_scene.ps1 -Smoke -NullRhi   # 描画なし（GPU の無い CI 用）
#   pwsh scripts\windows\ship_scene.ps1 -Open             # エディタで開く
#
# 先に scripts\windows\setup_build.ps1 でビルドしておく。
[CmdletBinding()]
param(
    [switch]$Build,
    [switch]$Smoke,
    [switch]$NullRhi,
    [switch]$Open,
    [double]$SmokeSec = 5.0,
    [string]$EngineRoot = (Join-Path $env:ProgramFiles 'Epic Games\UE_5.8'),
    [int]$ResX = 1280,
    [int]$ResY = 720
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$uproject = Join-Path $root 'HakoniwaShip.uproject'
$editor = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
$map = '/Game/Ship/Levels/ShipSea'
if (-not (Test-Path $editor)) { throw "見つかりません: $editor" }
# 実行時の shakoc.dll と共有メモリの設定は船のもの（shipnavi\build\shakoc-dll・shipnavi\runtime の Z:/mmap_ship）。
# ★ 箱庭の Windows インストーラが HAKO_CORE_DLL_PATH・HAKO_CONFIG_PATH を全体の環境変数に入れている（古い SDK）ので、ここで上書きする
$shipnavi = (Resolve-Path (Join-Path $root '..')).Path
$env:HAKO_CORE_DLL_PATH = Join-Path $shipnavi 'build\shakoc-dll\bin'
$env:HAKO_CONFIG_PATH = Join-Path $shipnavi 'runtime\windows-msvc-x64\cpp_core_config.json'
foreach ($f in @((Join-Path $env:HAKO_CORE_DLL_PATH 'shakoc.dll'), $env:HAKO_CONFIG_PATH)) { if (-not (Test-Path $f)) { throw "見つかりません: $f" } }
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$logDir = Join-Path $root 'Saved\ShipLogs'
New-Item -ItemType Directory -Force $logDir | Out-Null

function Invoke-Editor([string[]]$Arguments, [string]$Log, [string]$Pattern) {
    Write-Host "[ship_scene] $editor $($Arguments -join ' ')"
    $p = Start-Process -FilePath $editor -ArgumentList ($Arguments + "-abslog=$Log") -PassThru -Wait
    Select-String -Path $Log -Pattern $Pattern | ForEach-Object { Write-Host "  $($_.Line -replace '^\[[^\]]*\]\[[^\]]*\]', '')" }
    if ($p.ExitCode -ne 0) { throw "Unreal が終了コード $($p.ExitCode) で終わりました（$Log）" }
}

if ($Build) {
    $log = Join-Path $logDir "build_$stamp.log"
    Invoke-Editor @($uproject, '-unattended', '-nosplash', '-ShipBuildScene', '-ShipQuit', "-ShipMap=$map") $log 'LogShipEditor'
    if (-not (Select-String -Path $log -Pattern 'Ship.BuildScene OK' -Quiet)) { throw "場面を作れませんでした（$log）" }
}

if ($Smoke) {
    $log = Join-Path $logDir "smoke_$stamp.log"
    $gameArgs = @($uproject, $map, '-game', '-unattended', '-nosplash', '-windowed', "-ResX=$ResX", "-ResY=$ResY", "-ShipSmoke=$SmokeSec")
    if ($NullRhi) { $gameArgs += '-nullrhi' }
    Invoke-Editor $gameArgs $log 'LogShipView|LogHakoniwa|Error'
    $ok = Select-String -Path $log -Pattern 'HAKONIWA_UNREAL_SHIP_BOOT_OK' | Select-Object -First 1
    if (-not $ok) { throw "起動の確認に失敗しました（$log）" }
    Write-Host "[ship_scene] $($ok.Line -replace '^.*(HAKONIWA_UNREAL_SHIP_BOOT_OK)', '$1')"
}

if ($Open) {
    Start-Process -FilePath $editor -ArgumentList @($uproject, $map)
}
