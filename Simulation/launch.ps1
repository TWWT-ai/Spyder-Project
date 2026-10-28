# Run with: .\launch.ps1     or     .\launch.ps1 --demo
$legPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $legPython)) {
    $legPython = 'python'
}
& $legPython (Join-Path $PSScriptRoot 'simulate.py') @args
exit $LASTEXITCODE
