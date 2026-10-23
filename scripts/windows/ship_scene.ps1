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

. (Join-Path $PSScriptRoot 'ship_env.ps1') -EngineRoot $EngineRoot   # shakoc.dll・HAKO_CONFIG_PATH（船のもの）
$uproject = $ShipUproject
$editor = $ShipEditor
$map = $ShipMap
$logDir = $ShipLogDir
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'

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
