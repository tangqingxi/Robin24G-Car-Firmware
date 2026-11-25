param(
    [string]$BaseRoot = 'E:\tmp\clean_split',
    [string]$SplitRoot = '',
    [string]$OutDir = 'E:\tmp'
)

# 一条命令完成基线/拆分整链回放和逐拍比较。
$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($SplitRoot)) {
    $SplitRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
}
$gcc = 'D:\app\Visual Studio Code2025\c\MinGW\bin\gcc.exe'
$here = $PSScriptRoot
$replay = Join-Path $here 'replay_imu_chain.c'

function Build-Chain([string]$root, [string]$name, [bool]$withImu)
{
    $car = Join-Path $root 'telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car'
    $sources = @($replay, (Join-Path $car 'FlightLoop.c'))
    if ($withImu) { $sources += (Join-Path $car 'ImuMeasure.c') }
    $sources += @(
        (Join-Path $car 'RcInput.c'),
        (Join-Path $car 'FlightSafety.c'),
        (Join-Path $car 'MotorMixer.c'),
        (Join-Path $car 'AttitudeControl.c'),
        (Join-Path $car 'RateControl.c'),
        (Join-Path $car 'UserEuler.c'),
        (Join-Path $car 'GroundDiag.c')
    )
    $incs = @(
        "-I$(Join-Path $root 'telink_b85m_platform_src1015\common')",
        "-I$(Join-Path $root 'telink_b85m_platform_src1015\demo\vendor\common\common')",
        "-I$car",
        "-I$(Join-Path $root 'telink_b85m_platform_src1015\chip\tc122x\drivers')"
    )
    & $gcc -std=c99 -O2 -fno-builtin -Werror=format `
        -DFLIGHT_HOST_TEST=1 -DFLIGHT_STARTUP_TEST=1 `
        -DCLOCK_SYS_CLOCK_HZ=24000000 -DC_GROUND_UART_DIAG=1 -fpack-struct `
        @incs @sources -o (Join-Path $OutDir "imu_chain_$name.exe")
    if ($LASTEXITCODE -ne 0) { throw "$name 编译失败" }
}

Build-Chain $BaseRoot 'base' $false
Build-Chain $SplitRoot 'split' $true
& (Join-Path $OutDir 'imu_chain_base.exe') > (Join-Path $OutDir 'imu_chain_base.csv') 2> (Join-Path $OutDir 'imu_chain_base.err')
if ($LASTEXITCODE -ne 0) { Get-Content (Join-Path $OutDir 'imu_chain_base.err'); throw '基线回放失败' }
& (Join-Path $OutDir 'imu_chain_split.exe') > (Join-Path $OutDir 'imu_chain_split.csv') 2> (Join-Path $OutDir 'imu_chain_split.err')
if ($LASTEXITCODE -ne 0) { Get-Content (Join-Path $OutDir 'imu_chain_split.err'); throw '拆分回放失败' }

$base = Get-Content (Join-Path $OutDir 'imu_chain_base.csv')
$split = Get-Content (Join-Path $OutDir 'imu_chain_split.csv')
$diff = Compare-Object $base $split
"base=$($base.Count) split=$($split.Count) diff=$($diff.Count)"
if ($diff.Count) {
    $diff | Select-Object -First 1
    exit 1
}
