param([int]$Port = 8080)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
python server.py --host 0.0.0.0 --port $Port

