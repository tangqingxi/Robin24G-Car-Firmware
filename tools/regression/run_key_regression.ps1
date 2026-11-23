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

    Invoke-Case 'flight_log' @(
        (Join-Path $Here 'test_flight_log.c'),
        (Join-Path $Car 'FlightLog.c')
    ) @('-DFLIGHT_LOG_HOST_TEST=1', '-DFLIGHT_LOG_LEGACY_WRITE_TEST=1', '-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000')

    $logDeps = @('FlightLoop.c', 'RcInput.c', 'FlightSafety.c', 'MotorMixer.c',
        'AttitudeControl.c', 'RateControl.c', 'UserEuler.c', 'FlightLog.c') |
        ForEach-Object { Join-Path $Car $_ }
    Invoke-Case 'flight_log_integration' (@(
        (Join-Path $Here 'test_flight_log_integration.c')) + $logDeps) @(
        '-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000',
        '-DFLIGHT_STARTUP_TEST=1', '-DFLIGHT_LOG_HOST_TEST=1', '-DFLIGHT_LOG_LEGACY_WRITE_TEST=1', '-fpack-struct')
    foreach ($case in @('preserve', 'full', 'locked', 'unlock', 'closing')) {
        Invoke-Case ('flight_log_states_' + $case) (@(
            (Join-Path $Here 'test_flight_log_states.c')) + $logDeps) @(
            '-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000',
            '-DFLIGHT_STARTUP_TEST=1', '-DFLIGHT_LOG_HOST_TEST=1', '-DFLIGHT_LOG_LEGACY_WRITE_TEST=1', '-fpack-struct') @($case)
    }

    Invoke-Case 'boot_validate' @(
        (Join-Path $Here 'test_boot_validate.c')
    ) @()

    Invoke-Case 'ram_log' @(
        (Join-Path $Here 'test_flight_ram_log.c'),
        (Join-Path $Car 'FlightLog.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DFLIGHT_LOG_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000', '-fpack-struct')

    Invoke-Case 'flash_ab' @(
        (Join-Path $Here 'test_flash_ab.c'),
        (Join-Path $Car 'FlightLog.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000',
        '-DC_FLIGHT_ARM_DEBUG=1', '-fpack-struct')

    Invoke-Case 'arm_debug_gate' @(
        (Join-Path $Here 'test_flight_arm_debug.c'),
        (Join-Path $Car 'FlightArmDebug.c')
    ) @('-DFLIGHT_HOST_TEST=1', '-DCLOCK_SYS_CLOCK_HZ=24000000',
        '-DC_FLIGHT_ARM_DEBUG=1', '-DARM_DBG_QDEPTH=3', '-fpack-struct')

    $env:PYTHONIOENCODING = 'utf-8'
    foreach ($test in @('test_arm_debug_monitor.py', 'test_package_bin.py', 'test_l400_monitor.py')) {
        python (Join-Path $Here $test)
        if ($LASTEXITCODE -ne 0) { throw ($test + ' failed') }
    }
    Write-Host 'RESULT OK'
}
finally {
    Pop-Location
    Remove-Item $Tmp -Recurse -Force -ErrorAction SilentlyContinue
}
