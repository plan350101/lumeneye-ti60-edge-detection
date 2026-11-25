# 提交前一键判读（PowerShell 包装）
# 用法: pwsh -File _final_check.ps1 [秒数]
$ErrorActionPreference = 'Stop'
$py = 'C:\Users\27401\.workbuddy\binaries\python\envs\default\Scripts\python.exe'
$script = Join-Path $PSScriptRoot '_final_check.py'
& $py $script @args
exit $LASTEXITCODE
