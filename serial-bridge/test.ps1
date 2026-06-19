$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Python = Join-Path $ScriptDir ".venv\Scripts\python.exe"

if (-not (Test-Path -LiteralPath $Python)) {
    Write-Error "Missing .venv. Run: python -m venv .venv; .\.venv\Scripts\python -m pip install -r requirements.txt"
}

$env:PYTHONDONTWRITEBYTECODE = "1"
& $Python -B (Join-Path $ScriptDir "test_protocol.py")
