# ============================================================================
#  _build_rtl.ps1  -  PowerShell port of _build_rtl.sh (no Git Bash on this box)
#
#  Four steps: efx_map -> efx_run_pt_unified.py --peri_file -> efx_pnr -> efx_pgm
#
#  Diffs vs the .sh:
#    * syncs 6 project files + ov5640_cfg_rom.v (each backed up as *.bak_prebuild)
#    * fail-fast: a failed step stops the build (never bitstream from stale data)
#    * prints SDC self-check, RAM10 and timing summary
#
#  Usage:  pwsh -File _build_rtl.ps1
#          pwsh -File _build_rtl.ps1 -SkipSync
# ============================================================================
param(
    [switch]$SkipSync
)

$ErrorActionPreference = 'Continue'
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}

$W   = 'C:\Users\27401\WorkBuddy\2026-09-12-12-05-44'
$E   = 'E:\efinity_project\104.2'
$TOP = 'camera_hdmi'
$LOG = Join-Path $W '_build_rtl.log'

# NOTE: forward slashes are mandatory. efx_pnr builds a Tcl path from
# EFINITY_HOME; backslashes get eaten as Tcl escapes.
$EFX     = 'F:/Download/efinity'
$env:EFINITY_HOME         = $EFX
$env:EFXPT_HOME           = "$EFX/pt"
$env:EFXIPM_HOME          = "$EFX/ipm"
$env:EFXPGM_HOME          = "$EFX/pgm"
$env:EFXDBG_HOME          = "$EFX/debugger"
$env:PYTHONHOME           = "$EFX/python311"
$env:PYTHONPATH           = "$EFX/pt/bin"
$env:EFINITY_USER_DIR     = 'C:/Users/27401/.efinity'
$env:EFINITY_USER_DIR_INI = 'C:/Users/27401/AppData/Local/efinity/user_dir.ini'
$EFX_PY  = "$EFX/python311/bin/python.exe"
$EFX_MAP = "$EFX/bin/efx_map.exe"
$EFX_PNR = "$EFX/bin/efx_pnr.exe"
$EFX_PGM = "$EFX/bin/efx_pgm.exe"

$lines = New-Object System.Collections.Generic.List[string]
function Log([string]$s) {
    Write-Host $s
    $lines.Add($s) | Out-Null
    Set-Content -Path $LOG -Value ($lines -join "`r`n") -Encoding UTF8
}

Log ("=== build start: {0} ===" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
Log ""

# ----------------------------------------------------------- sync 6 + ROM
$sync = @(
    'camera_hdmi.v',
    'ddr_frame_buffer.v',
    'camera_hdmi.sdc',
    'camera_hdmi.peri.xml',
    'camera_hdmi.isf',
    'camera_hdmi.xml',
    'median3x3.v',
    'canny_core.v',
    'shape_detect.v',
    'thr_ctrl.v',
    'ov5640_cfg_rom.v'
)
if (-not $SkipSync) {
    Log "########## Step 0/4 sync 6 files + config ROM ##########"
    foreach ($f in $sync) {
        $src = Join-Path $W $f
        $dst = Join-Path $E $f
        if (-not (Test-Path $src)) { Log ("  [SKIP] not in workspace: {0}" -f $f); continue }
        if (-not (Test-Path $dst)) { Log ("  [ERR ] not in E: project: {0}" -f $f); continue }
        $bak = "$dst.bak_prebuild"
        if (-not (Test-Path $bak)) { Copy-Item $dst $bak -Force }
        Copy-Item $src $dst -Force
        $h1 = (Get-FileHash $src -Algorithm MD5).Hash
        $h2 = (Get-FileHash $dst -Algorithm MD5).Hash
        Log ("  {0,-26} {1}" -f $f, ($(if ($h1 -eq $h2) { 'OK' } else { 'HASH MISMATCH!' })))
    }
    Log ""
}

# ----------------------------------------------------------- SDC self-check
Log "########## Step 0b/4 SDC self-check ##########"
$sdcTxt = [System.IO.File]::ReadAllLines((Join-Path $E 'camera_hdmi.sdc'), [System.Text.Encoding]::UTF8)
$nLine = $sdcTxt.Count
$nClk  = ($sdcTxt | Where-Object { $_ -match '^\s*create_clock' }).Count
$nFp   = ($sdcTxt | Where-Object { $_ -match 'set_false_path' }).Count
$nX    = ($sdcTxt | Where-Object { $_ -match 'get_clocks \{clk_sys\}\] -to \[get_clocks \{clk_pixel\}\]' }).Count
Log ("  SDC lines          : {0}   (expected 171)" -f $nLine)
Log ("  create_clock       : {0}   (expected 9)"  -f $nClk)
Log ("  set_false_path     : {0}   (expected 9)"  -f $nFp)
Log ("  clk_sys->clk_pixel : {0}   (expected 1)"  -f $nX)
Log ""

Push-Location $E
New-Item -ItemType Directory -Force -Path outflow, work_syn, work_pt, work_pnr | Out-Null

# ----------------------------------------------------------- step 1/4
Log "########## Step 1/4 efx_map ##########"
$t0 = Get-Date
& $EFX_MAP --project $TOP --root $TOP `
    --write-efx-verilog (Join-Path $E "outflow\$TOP.map.v") `
    --write-premap-module (Join-Path $E "outflow\$TOP.elab.vdb") `
    --binary-db (Join-Path $E "outflow\$TOP.vdb") `
    --device Ti60F225 --family Titanium `
    --veri_option "verilog_mode=verilog_2k,vhdl_mode=vhdl_2008" `
    --work-dir (Join-Path $E 'work_syn') --output-dir (Join-Path $E 'outflow') `
    --prj (Join-Path $E "$TOP.xml") --I $E *> (Join-Path $W '_map_raw.log')
$rc = $LASTEXITCODE
Log ("  exit={0}  ({1:N1}s)" -f $rc, ((Get-Date) - $t0).TotalSeconds)
if ($rc -ne 0) { Log "  [FATAL] step failed - stopping (no stale bitstream)"; Pop-Location; exit 1 }
Log ""

# ----------------------------------------------------------- step 2/4
Log "########## Step 2/4 PT (--peri_file) ##########"
$t0 = Get-Date
& $EFX_PY "$EFX/scripts/efx_run_pt_unified.py" `
    $TOP Titanium Ti60F225 `
    --peri_file (Join-Path $E "$TOP.peri.xml") `
    --output_dir (Join-Path $E 'outflow') --work_dir (Join-Path $E 'work_pt') `
    --design_dir $E *> (Join-Path $W '_pt_raw.log')
$rc = $LASTEXITCODE
Log ("  exit={0}  ({1:N1}s)" -f $rc, ((Get-Date) - $t0).TotalSeconds)
if ($rc -ne 0) { Log "  [FATAL] step failed - stopping (no stale bitstream)"; Pop-Location; exit 1 }
Log ""

# ----------------------------------------------------------- step 3/4
Log "########## Step 3/4 efx_pnr ##########"
$t0 = Get-Date
& $EFX_PNR --prj (Join-Path $E "$TOP.xml") --circuit $TOP `
    --family Titanium --device Ti60F225 --operating_conditions C4 `
    --pack --place --route `
    --vdb_file (Join-Path $E "outflow\$TOP.vdb") --use_vdb_file on `
    --place_file (Join-Path $E "outflow\$TOP.place") `
    --route_file (Join-Path $E "outflow\$TOP.troutingtraces") `
    --sdc_file (Join-Path $E "$TOP.sdc") --sync_file (Join-Path $E "outflow\$TOP.interface.csv") `
    --seed 1 --placer_effort_level 2 --max_threads -1 --print_critical_path 10 `
    --beneficial_skew on --work_dir (Join-Path $E 'work_pnr') --output_dir (Join-Path $E 'outflow') `
    --timing_analysis on *> (Join-Path $W '_pnr_raw.log')
$rc = $LASTEXITCODE
Log ("  exit={0}  ({1:N1}s)" -f $rc, ((Get-Date) - $t0).TotalSeconds)
if ($rc -ne 0) { Log "  [FATAL] step failed - stopping (no stale bitstream)"; Pop-Location; exit 1 }
Log ""

# ----------------------------------------------------------- step 4/4
Log "########## Step 4/4 efx_pgm ##########"
$t0 = Get-Date
& $EFX_PGM --source (Join-Path $E "work_pnr\$TOP.lbf") `
    --dest (Join-Path $E "outflow\$TOP.hex") --device Ti60F225 --family Titanium `
    --periph (Join-Path $E "outflow\$TOP.lpf") `
    --interface_designer_settings (Join-Path $E "outflow\${TOP}_or.ini") `
    --enable_external_master_clock off --oscillator_clock_divider DIV8 `
    --active_capture_clk_edge posedge --spi_low_power_mode on --io_weak_pullup on `
    --enable_roms smart --mode active --width 1 --release_tri_then_reset on `
    --bitstream_compression on --jtag_usercode 0xFFFFFFFF *> (Join-Path $W '_pgm_raw.log')
$rc = $LASTEXITCODE
Log ("  exit={0}  ({1:N1}s)" -f $rc, ((Get-Date) - $t0).TotalSeconds)
if ($rc -ne 0) { Log "  [FATAL] step failed - stopping"; Pop-Location; exit 1 }
Log ""

Pop-Location

# ----------------------------------------------------------- summary
Log "########## artifacts / baseline metrics ##########"
$bit = Join-Path $E "outflow\$TOP.bit"
if (Test-Path $bit) {
    $b = Get-Item $bit
    Log ("  {0}  {1} B  {2}" -f $b.Name, $b.Length, $b.LastWriteTime.ToString('yyyy-MM-dd HH:mm:ss'))
} else { Log "  NO .bit !!" }

$res = Join-Path $E "outflow\$TOP.res.csv"
if (Test-Path $res) {
    Select-String -Path $res -Pattern 'RAM10' | Select-Object -First 4 |
        ForEach-Object { Log ("  res.csv: " + $_.Line.Trim()) }
}

$tr = Join-Path $E "outflow\$TOP.timing.rpt"
if (Test-Path $tr) {
    Log "  timing.rpt:"
    Select-String -Path $tr -Pattern 'Slack|VIOLATION|WNS|WHS' |
        Select-Object -First 30 | ForEach-Object { Log ("    " + $_.Line.Trim()) }
}

Log ""
Log ("=== build end: {0} ===" -f (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
