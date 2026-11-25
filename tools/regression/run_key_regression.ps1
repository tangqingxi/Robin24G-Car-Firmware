# 关键回归：日志、CRC、boot 校验和 Python 工具。
# 运行：powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
$ErrorActionPreference = 'Stop'
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Root = Resolve-Path (Join-Path $Here '..\..')
$Car = Join-Path $Root 'telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car'
$Common = Join-Path $Root 'telink_b85m_platform_src1015\common'
$Drv = Join-Path $Root 'telink_b85m_platform_src1015\chip\tc122x\drivers'
$FwCommon = Join-Path $Root 'telink_b85m_platform_src1015\demo\vendor\common\common'
$Boot = Join-Path $Root 'telink_b85m_platform_src1015\demo\vendor\Robin_24G_Bootloader'
$Tmp = Join-Path $env:TEMP ('robin_regression_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force $Tmp | Out-Null
Push-Location $Tmp
Copy-Item (Join-Path $Here 'fixtures\baro_real_v4_20261124.bin') `
    (Join-Path $Tmp 'v4_actual.bin')

$Gcc = (Get-Command gcc -ErrorAction SilentlyContinue).Source
if (-not $Gcc) {
    $Gcc = 'D:\app\Visual Studio Code2025\c\MinGW\bin\gcc.exe'
}
if (-not (Test-Path $Gcc)) {
    throw 'gcc not found'
}

function Invoke-Case([string]$Name, [string[]]$Sources, [string[]]$Definitions, [string[]]$RunArgs = @()) {
    $exe = Join-Path $Tmp ($Name + '.exe')
    $gccArgs = @('-std=c99', '-O2', '-fno-builtin') + $Definitions +
        @("-I$Common", "-I$FwCommon", "-I$Car", "-I$Boot", "-I$Drv") + $Sources + @('-o', $exe)
    & $Gcc @gccArgs
    if ($LASTEXITCODE -ne 0) { throw ($Name + ' compile failed') }
    & $exe @RunArgs
    if ($LASTEXITCODE -ne 0) { throw ($Name + ' run failed') }
}

try {
    Invoke-Case 'flash_crc' @(
        (Join-Path $Here 'test_flash_fw_check.c'),
        (Join-Path $Root 'telink_b85m_platform_src1015\demo\vendor\common\common\flash_fw_check.c')
    ) @('-DFLASH_FW_CHECK_HOST_TEST=1')

    Invoke-Case 'boot_validate' @(
        (Join-Path $Here 'test_boot_validate.c')
    ) @()

    Invoke-Case 'flight_order' @(
        (Join-Path $Here 'test_flight_order.c'),
        (Join-Path $Car 'FlightLoop.c'),
        (Join-Path $Car 'AttitudeEstimate.c'),
        (Join-Path $Car 'ImuMeasure.c'),
        (Join-Path $Car 'RcInput.c'),
        (Join-Path $Car 'FlightSafety.c'),
        (Join-Path $Car 'MotorMixer.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000',
        '-DFLIGHT_ORDER_TEST=1', '-fpack-struct')

    Invoke-Case 'ground_diag' @(
        (Join-Path $Here 'test_ground_diag.c'),
        (Join-Path $Car 'GroundDiag.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct')

    Invoke-Case 'yaw_symmetry' @(
        (Join-Path $Here 'test_yaw_symmetry.c'),
        (Join-Path $Car 'RateControl.c'),
        (Join-Path $Car 'MotorMixer.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct')

    Invoke-Case 'lpc2623_ready' @(
        (Join-Path $Here 'test_lpc2623_ready.c'),
        (Join-Path $Car 'PressSen_Lpc2623_Ready.c')
    ) @('-DBARO_BDT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000')

    Invoke-Case 'baro_bdt' @(
        (Join-Path $Here 'test_baro_bdt.c'),
        (Join-Path $Car 'BaroBdt.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DBARO_BDT_HOST_TEST=1',
        '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct')

    Invoke-Case 'imu_measure' @(
        (Join-Path $Here 'test_imu_measure.c'),
        (Join-Path $Car 'ImuMeasure.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000')

    Invoke-Case 'baro_fixed' @(
        (Join-Path $Here 'test_baro_fixed.c'),
        (Join-Path $Car 'BaroFixed.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DBARO_BDT_HOST_TEST=1',
        '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct',
        '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections')

    Invoke-Case 'baro_temp' @(
        (Join-Path $Here 'test_baro_temp_bdt.c'),
        (Join-Path $Car 'BaroTempBdt.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DBARO_BDT_HOST_TEST=1',
        '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct')

    Invoke-Case 'baro_fixed_diag' @(
        (Join-Path $Here 'test_baro_fixed_bdt.c'),
        (Join-Path $Car 'BaroFixedBdt.c'),
        (Join-Path $Car 'BaroFixed.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DBARO_BDT_HOST_TEST=1',
        '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct',
        '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections')

    Invoke-Case 'baro_fixed_actual' @(
        (Join-Path $Here 'test_baro_fixed_v4.c'),
        (Join-Path $Car 'BaroFixed.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct')

    Invoke-Case 'baro_measure' @(
        (Join-Path $Here 'test_baro_measure.c'),
        (Join-Path $Car 'BaroMeasure.c'),
        (Join-Path $Car 'BaroFixed.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct')

    Invoke-Case 'baro_drift' @(
        (Join-Path $Here 'test_baro_bdt.c'),
        (Join-Path $Car 'BaroBdt.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DBARO_BDT_HOST_TEST=1',
        '-DBARO_BDT_SAMPLE_MS=500', '-DCLOCK_SYS_CLOCK_HZ=24000000',
        '-fpack-struct')

    $startupSources = @(
        (Join-Path $Here 'test_flight_startup.c'),
        (Join-Path $Car 'FlightLoop.c'),
        (Join-Path $Car 'AttitudeEstimate.c'),
        (Join-Path $Car 'ImuMeasure.c'),
        (Join-Path $Car 'RcInput.c'),
        (Join-Path $Car 'FlightSafety.c'),
        (Join-Path $Car 'MotorMixer.c'),
        (Join-Path $Car 'AttitudeControl.c'),
        (Join-Path $Car 'RateControl.c'),
        (Join-Path $Car 'UserEuler.c'),
        (Join-Path $Car 'GroundDiag.c')
    )
    $startupFlags = @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000',
        '-DC_GROUND_UART_DIAG=1', '-DFLIGHT_STARTUP_TEST=1', '-fpack-struct')
    Invoke-Case 'flight_startup_ground' $startupSources $startupFlags
    Invoke-Case 'flight_startup_missing_baro' ($startupSources + @(
        (Join-Path $Here 'test_baro_measure.c'),
        (Join-Path $Car 'BaroMeasure.c'),
        (Join-Path $Car 'BaroFixed.c')
    )) ($startupFlags + @('-DC_FLIGHT_BARO_EN=1', '-DBARO_MEASURE_FLIGHT_REPLAY=1'))




    $env:PYTHONIOENCODING = 'utf-8'
    foreach ($test in @('test_baro_export.py', 'test_baro_fixed_v4_export.py', 'test_package_bin.py')) {
        python (Join-Path $Here $test)
        if ($LASTEXITCODE -ne 0) { throw ($test + ' failed') }
    }
    Write-Host 'RESULT OK'
}
finally {
    Pop-Location
    Remove-Item $Tmp -Recurse -Force -ErrorAction SilentlyContinue
}
