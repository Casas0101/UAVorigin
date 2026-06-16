# verify_raw_minimal.ps1
#
# Windows 11 下第一环最简原始数据中间件验收脚本.
# 依据: 工程文档/低智能AI_Windows11无仿真无C++生成指南.md § 4.
#
# 步骤:
#   1. python -m pytest -q
#   2. python tools/generate_mock_raw_frames.py --output-dir outputs/raw_frames --count 5
#   3. python -m pytest -q   (二次, 确认生成过程未破坏测试)
#
# 任意步骤失败立即退出, 非零返回.

[CmdletBinding()]
param(
    [int] $Count = 5,
    [switch] $Overwrite
)

$ErrorActionPreference = "Stop"

function Run-Step {
    param(
        [string] $Label,
        [string] $Display,
        [scriptblock] $Command
    )
    Write-Host ""
    Write-Host "==> $Label" -ForegroundColor Cyan
    Write-Host "    $Display" -ForegroundColor DarkGray
    & $Command
    $exitCode = if ($null -eq $LASTEXITCODE) { 0 } else { $LASTEXITCODE }
    if ($exitCode -ne 0) {
        Write-Host "    step FAILED with exit code $exitCode" -ForegroundColor Red
        exit $exitCode
    }
    Write-Host "    step OK" -ForegroundColor Green
}

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Resolve-Path (Join-Path $here "..")
$pytestTemp = Join-Path $root.Path ".pytest-tmp"
New-Item -ItemType Directory -Force -Path $pytestTemp | Out-Null
$env:TMP = $pytestTemp
$env:TEMP = $pytestTemp
$env:PYTEST_DEBUG_TEMPROOT = $pytestTemp

Push-Location $root
try {
    Run-Step -Label "Step 1: pytest" `
             -Display "python -m pytest -q" `
             -Command { python -m pytest -q }

    $genArgs = @(
        "tools/generate_mock_raw_frames.py",
        "--output-dir", "outputs/raw_frames",
        "--count", $Count
    )
    if ($Overwrite) {
        $genArgs += "--overwrite"
    }
    $genCmd = "python " + ($genArgs -join " ")
    Run-Step -Label "Step 2: generate mock raw frames" `
             -Display $genCmd `
             -Command { python @genArgs }

    Run-Step -Label "Step 3: pytest (post-generation)" `
             -Display "python -m pytest -q" `
             -Command { python -m pytest -q }

    Write-Host ""
    Write-Host "All steps passed." -ForegroundColor Green
    exit 0
}
finally {
    Pop-Location
}
