# ship_scene.ps1 — 海の場面（SHP-24）を組む・起動を確認する・エディタで開く。hakoniwa-unreal-rover の rover_scene.ps1 と同じ形。
#
#   pwsh scripts\windows\ship_scene.ps1 -Build            # /Game/Ship/Levels/ShipSea を作り直す（clone の後に必須。エディタが自動で開いて閉じる）
#   pwsh scripts\windows\ship_scene.ps1 -Smoke            # -game で開き、数秒描いて HAKONIWA_UNREAL_SHIP_BOOT_OK を確認する
#                                                         # （波面の計算と Godot の参照値の一致、描いた海面の高さも確かめる。-Shot で撮影）
#   pwsh scripts\windows\ship_scene.ps1 -Smoke -NullRhi   # 描画なし（GPU の無い CI 用）
#   pwsh scripts\windows\ship_scene.ps1 -Open             # エディタで開く
#
# 先に scripts\windows\setup_build.ps1 でビルドしておく。
[CmdletBinding()]
param(
    [switch]$Build,
    [switch]$Smoke,
    [switch]$NullRhi,
    [switch]$Shot,                # -Smoke の終わりに自船を追うカメラで撮る（Saved\ShipLogs\smoke_<時刻>.png）
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
    # 波面の計算（wave_spectrum.hpp）を Godot の参照値（Python 実装の値）と比べる。描画があれば、描いた海面の高さも確かめる（Step 3）
    $parityRef = Join-Path $ShipNavi 'hakoniwa-godot-ship\tests\unit\wave_parity\reference.json'
    $gameArgs = @($uproject, $map, '-game', '-unattended', '-nosplash', '-windowed', "-ResX=$ResX", "-ResY=$ResY", "-ShipSmoke=$SmokeSec",
        "-ShipWaveParity=$parityRef")
    if ($NullRhi) { $gameArgs += '-nullrhi' } else { $gameArgs += '-ShipSurfaceCheck' }
    if ($Shot) { $gameArgs += @('-ShipChase', "-ShipShot=$(Join-Path $logDir "smoke_$stamp.png")") }
    Invoke-Editor $gameArgs $log 'LogShipView|LogHakoniwa|LogShipOcean|Error'
    $ok = Select-String -Path $log -Pattern 'HAKONIWA_UNREAL_SHIP_BOOT_OK' | Select-Object -First 1
    if (-not $ok) { throw "起動の確認に失敗しました（$log）" }
    if (-not (Select-String -Path $log -Pattern 'HAKONIWA_UNREAL_SHIP_WAVE_PARITY_OK' -Quiet)) { throw "波面の計算が参照値と一致しません（$log）" }
    if (-not $NullRhi) {
        # 各回の検証（SHIP_SEA surface check <n>: ... misses <m>, max err <e> m）がすべて許容内か
        $checks = @(Select-String -Path $log -Pattern 'SHIP_SEA surface check \d+:.*misses (\d+), max err ([0-9.e+-]+) m')
        $bad = @($checks | Where-Object { [int]$_.Matches[0].Groups[1].Value -gt 0 -or [double]$_.Matches[0].Groups[2].Value -gt 5e-3 })
        if ($checks.Count -eq 0 -or $bad.Count -gt 0) { throw "描いた海面の高さが物理の波面と一致しません（$log）" }
        Write-Host "[ship_scene] surface checks: $($checks.Count) OK"
    }
    Write-Host "[ship_scene] $($ok.Line -replace '^.*(HAKONIWA_UNREAL_SHIP_BOOT_OK)', '$1')"
}

if ($Open) {
    Start-Process -FilePath $editor -ArgumentList @($uproject, $map)
}
