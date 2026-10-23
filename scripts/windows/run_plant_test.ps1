# run_plant_test.ps1 — 実物理との結合テスト（SHP-24 Step 2・3）。hakoniwa-godot-ship の tools/run_plant_test.ps1 と同じ構成で、
# ビューアを Unreal にしたもの：conductor → ship_plant ＋ 外乱パブリッシャ → Unreal（-game）。
#
#   pwsh scripts\windows\run_plant_test.ps1 [-CheckSec 60] [-TimeoutSec 300] [-Screenshot]
#
# ship_plant（hakoniwa-mujoco-ship、事前に cmake --build build --config Release）が全船の pos・OwnShip の joint_states を書く。
# Unreal は -ShipSync -ShipHakoStart で箱庭に登録して開始を要求し、物理時刻で CheckSec 秒受信したら
#   1) 受信数と、アクタの位置・軸・関節の原点・関節角が PDU どおりか（AShipFleet の検証）、
#      海面（Step 3）：波浪パラメータ・海面の時刻が物理時刻か・船体の上下動と海面の相関と水線・描いた海面の高さ → HAKONIWA_UNREAL_SHIP_SYNC_OK
#   2) アクタから読み戻した姿勢（-ShipSamples）を ship_plant の --trace と照合（tools\compare_plant_trace.py）→ HAKONIWA_UNREAL_SHIP_TRACE_OK
#   3) 船載カメラ（SHP-25）：受け手のアセット（tools\check_camera_pdu.py）が 2 台の画像 PDU を読み、数・時刻・形式を確かめ（CAMERA_PDU_OK）、
#      船橋カメラの水平線を同じ時刻の姿勢から求めた位置と比べる（tools\compare_camera_horizon.py → CAMERA_HORIZON_OK）
# 両方 OK なら HAKONIWA_UNREAL_SHIP_PLANT_OK を出して 0 で終わる。ログは Saved\ShipLogs\plant_<時刻>\。
# -Screenshot を付けると終了の少し前に画面を撮る（Saved\ShipLogs\plant_<時刻>\view.png、目視確認用）。
[CmdletBinding()]
param(
    [double]$CheckSec = 60.0,
    [int]$TimeoutSec = 300,
    [switch]$Screenshot,
    [string[]]$ExtraArgs = @(),   # Unreal に足す引数（診断用、例：'-ExecCmds="r.Shadow.Virtual.Enable 0"'）
    [string]$Scenario = '',
    [string]$EngineRoot = (Join-Path $env:ProgramFiles 'Epic Games\UE_5.8'),
    [int]$ResX = 1280,
    [int]$ResY = 720
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'ship_env.ps1') -EngineRoot $EngineRoot

$PlantRepo = Join-Path $ShipNavi 'hakoniwa-mujoco-ship'
$Plant = Join-Path $PlantRepo 'build\Release\ship_plant.exe'
if (-not $Scenario) { $Scenario = Join-Path $PlantRepo 'config\plant\fleet_encounter.json' }
$EnvPub = Join-Path $PlantRepo 'tools\publish_disturbance.py'
$MujocoBin = Join-Path $PlantRepo 'deps\mujoco-3.9.0\bin'
$Conductor = Join-Path $ShipNavi 'build\hakoniwa-godot\build\tools\hako_conductor_runner.exe'
$Python = Join-Path $ShipNavi 'sdk\windows-msvc-x64\python-venv\Scripts\python.exe'
$PduConfig = Join-Path $ShipNavi 'pdu_config_ship.json'
$Compare = Join-Path $ShipRoot 'tools\compare_plant_trace.py'
$CamCheck = Join-Path $ShipRoot 'tools\check_camera_pdu.py'
$CamHorizon = Join-Path $ShipRoot 'tools\compare_camera_horizon.py'
$FleetUe = Join-Path $ShipRoot 'Config\Fleet\fleet_ue.json'
foreach ($p in $Plant, $Scenario, $EnvPub, $MujocoBin, $Conductor, $Python, $PduConfig, $Compare, $CamCheck, $CamHorizon, $FleetUe) { if (-not (Test-Path $p)) { throw "見つかりません: $p" } }
$runDir = Join-Path $ShipLogDir ("plant_" + (Get-Date -Format 'yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Force $runDir | Out-Null
$env:PATH = "$MujocoBin;" + $env:PATH
$trace = Join-Path $runDir 'plant_trace.csv'
$samples = Join-Path $runDir 'ue_samples.csv'
$ueLog = Join-Path $runDir 'unreal.log'

$procs = @()
function Start-Logged($name, $exe, $argList) {
    $p = Start-Process $exe -ArgumentList $argList -PassThru -NoNewWindow `
        -RedirectStandardOutput (Join-Path $runDir "$name.log") -RedirectStandardError (Join-Path $runDir "$name.err.log")
    $script:procs += $p
    return $p
}

$syncOk = $false
$dllOk = $false
try {
    # ★ 箱庭の共有メモリ（HAKO_MMAP_DIR、Z:/mmap_ship）は前の実行の大きさのまま残り、PDU の表が大きくなっても広がらない
    #   （SHP-25 で画像の PDU を足したとき、Unreal が書き込みで落ちた）。前の箱庭が動いていなければ始める前に消す
    if (Get-Process -Name 'hako_conductor_runner', 'ship_plant' -ErrorAction SilentlyContinue) { throw '前の箱庭（conductor / ship_plant）がまだ動いています' }
    if ($env:HAKO_MMAP_DIR -and (Test-Path $env:HAKO_MMAP_DIR)) {
        Get-ChildItem -Path $env:HAKO_MMAP_DIR -File | Where-Object { $_.Name -like 'mmap-*.bin' -or $_.Name -like '*_init.lock' } | Remove-Item -Force
    }
    # conductor の待ち時間は Unreal の起動（シェーダの読み込み）を含めて長めに
    Start-Logged 'conductor' $Conductor @('--delta-usec', '20000', '--max-delay-usec', '100000', '--sleep-sec', "$TimeoutSec") | Out-Null
    Start-Sleep -Seconds 2
    Start-Logged 'plant' $Plant @($Scenario, '--trace', $trace) | Out-Null
    Start-Logged 'env' $Python @('-u', $EnvPub, $PduConfig, $Scenario) | Out-Null
    # 船載カメラの受け手（Unreal が書き始めてから CheckSec − 5 s の物理時刻だけ読む）
    $camDir = Join-Path $runDir 'camera'
    Start-Logged 'camera' $Python @('-u', $CamCheck, $PduConfig, $FleetUe, $camDir, '--duration', "$([Math]::Max(10, $CheckSec - 5))") | Out-Null
    Start-Sleep -Seconds 3
    $ueArgs = @($ShipUproject, $ShipMap, '-game', '-unattended', '-nosplash', '-windowed', "-ResX=$ResX", "-ResY=$ResY",
        '-ShipSync', '-ShipHakoStart', "-ShipSyncCheck=$CheckSec", "-ShipSamples=$samples", "-abslog=$ueLog")
    $ueArgs += $ExtraArgs
    if ($Screenshot) { $ueArgs += "-ShipShot=$(Join-Path $runDir 'view.png')" }
    Write-Host "[plant-test] $ShipEditor $($ueArgs -join ' ')"
    $ue = Start-Process -FilePath $ShipEditor -ArgumentList $ueArgs -PassThru
    $script:procs += $ue
    # 読み込まれた shakoc.dll を確かめる（船の DLL 1 つだけのはず。古い SDK の DLL を読むと共有メモリの形式が合わないことがある）
    $shakoc = @()
    for ($i = 0; $i -lt 60 -and -not $ue.HasExited -and $shakoc.Count -eq 0; $i++) {
        Start-Sleep -Seconds 1
        try { $shakoc = @((Get-Process -Id $ue.Id).Modules | Where-Object { $_.ModuleName -ieq 'shakoc.dll' } | ForEach-Object FileName) } catch { }
    }
    Write-Host "[plant-test] shakoc.dll in Unreal: $($shakoc -join ', ')"
    $wantDll = (Resolve-Path (Join-Path $env:HAKO_CORE_DLL_PATH 'shakoc.dll')).Path
    $dllOk = ($shakoc.Count -eq 1) -and ($shakoc[0] -ieq $wantDll)
    if (-not $ue.WaitForExit($TimeoutSec * 1000)) {
        Write-Host '[plant-test] Unreal timeout'
    }
} finally {
    foreach ($p in $procs) { if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } }
}

foreach ($name in 'conductor', 'plant', 'env', 'camera') {
    Write-Host "----- $name -----"
    Get-Content (Join-Path $runDir "$name.log"), (Join-Path $runDir "$name.err.log") -ErrorAction SilentlyContinue |
        Where-Object { $_ -and $_ -notmatch '\x1b\[' -and $_ -notmatch 'DEBUG|Log message truncated' } | Select-Object -Last 6
}
Write-Host '----- unreal -----'
if (Test-Path $ueLog) {
    Select-String -Path $ueLog -Pattern 'LogShipFleet|LogShipOcean: wave|HAKONIWA_UNREAL_SHIP|hako_asset_init|Hako Event' |
        Select-Object -Last 32 | ForEach-Object { Write-Host "  $($_.Line -replace '^\[[^\]]*\]\[[^\]]*\]', '')" }
    $syncOk = [bool](Select-String -Path $ueLog -Pattern 'HAKONIWA_UNREAL_SHIP_SYNC_OK' -Quiet)
}
Write-Host '----- trace -----'
$traceOk = $false
if ((Test-Path $trace) -and (Test-Path $samples)) {
    & $Python -I $Compare $trace $samples
    $traceOk = ($LASTEXITCODE -eq 0)
} else {
    Write-Host "trace or samples missing: $trace / $samples"
}
Write-Host '----- camera -----'
$camOk = [bool](Select-String -Path (Join-Path $runDir 'camera.log') -Pattern '^CAMERA_PDU_OK' -Quiet -ErrorAction SilentlyContinue)
Select-String -Path (Join-Path $runDir 'camera.log') -Pattern '^  camera_|^CAMERA_PDU' -ErrorAction SilentlyContinue | ForEach-Object { Write-Host $_.Line }
$horizonOk = $false
if (Test-Path $trace) {
    & $Python -I $CamHorizon $trace $FleetUe $camDir | Select-Object -Last 4
    $horizonOk = ($LASTEXITCODE -eq 0)
}
if ($syncOk -and $traceOk -and $dllOk -and $camOk -and $horizonOk) {
    Write-Host "HAKONIWA_UNREAL_SHIP_PLANT_OK ($runDir)"
    exit 0
}
Write-Host "HAKONIWA_UNREAL_SHIP_PLANT_FAILED sync=$syncOk trace=$traceOk shakoc=$dllOk camera=$camOk horizon=$horizonOk ($runDir)"
exit 1
