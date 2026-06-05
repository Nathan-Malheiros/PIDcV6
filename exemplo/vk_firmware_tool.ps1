#Requires -Version 5.1
# VitalKareV2 Firmware Tool
# Launched by vk_firmware_tool.bat which sources the ESP-IDF profile first.

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ToolDir  = Split-Path -Parent $MyInvocation.MyCommand.Path
$FwRoot   = (Resolve-Path (Join-Path $ToolDir '..')).Path
$BuildDir = Join-Path $FwRoot 'build'
$LogDir   = Join-Path $ToolDir 'logs'
$CfgFile  = Join-Path $ToolDir '.vk_config'
if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory $LogDir -Force | Out-Null }
$LogFile  = Join-Path $LogDir 'vk_tool.log'

$script:Port = ''
$script:Baud = 115200

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

function Log([string]$m) {
    $ts = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
    Add-Content $LogFile "[$ts] $m" -ErrorAction SilentlyContinue
}

function Header([string]$title = '') {
    Clear-Host
    Write-Host ''
    Write-Host '  ================================================' -ForegroundColor Magenta
    Write-Host '  VitalKareV2 / KIIRA V2  --  Firmware Tool' -ForegroundColor White
    if ($title) { Write-Host "  $title" -ForegroundColor Cyan }
    Write-Host '  ================================================' -ForegroundColor Magenta
    Write-Host ''
}

function StatusLine {
    $idfOk = [bool](Get-Command idf.py -ErrorAction SilentlyContinue)
    Write-Host "  Root   : $FwRoot" -ForegroundColor Cyan
    $portLabel = if ($script:Port) { $script:Port } else { 'auto-detect on use' }
    Write-Host "  COM    : $portLabel" -ForegroundColor Cyan

    # Show current device identity from sdkconfig.defaults (quick glance before flash)
    $def = Join-Path $FwRoot 'sdkconfig.defaults'
    if (Test-Path $def) {
        $dc = [System.IO.File]::ReadAllText($def, [System.Text.Encoding]::UTF8)
        $devId   = if ($dc -match 'CONFIG_VK_DEVICE_ID="([^"]*)"')         { $matches[1] } else { '?' }
        $devWard = if ($dc -match 'CONFIG_VK_DEVICE_WARD="([^"]*)"')       { $matches[1] } else { '?' }
        $devBed  = if ($dc -match 'CONFIG_VK_DEVICE_BED_NUMBER="([^"]*)"') { $matches[1] } else { '?' }
        Write-Host "  Device : $devId  (Ala $devWard  Leito $devBed)" -ForegroundColor Yellow
    }

    if ($idfOk) { Write-Host '  IDF    : available' -ForegroundColor Green }
    else         { Write-Host '  IDF    : NOT FOUND -- source ESP-IDF profile first' -ForegroundColor Red }
    Write-Host ''
}

function Pause {
    Write-Host ''
    Write-Host '  Press Enter to return to menu...' -ForegroundColor DarkGray
    $null = Read-Host
}

function LoadConfig {
    if (-not (Test-Path $CfgFile)) { return }
    Get-Content $CfgFile | ForEach-Object {
        if ($_ -match '^PORT=(.+)$') { $script:Port = $matches[1] }
        if ($_ -match '^BAUD=(\d+)$') { $script:Baud = [int]$matches[1] }
    }
}

function SaveConfig {
    "PORT=$($script:Port)`nBAUD=$($script:Baud)" |
        Set-Content $CfgFile -Encoding ASCII -ErrorAction SilentlyContinue
}

function GetEspPort {
    try {
        Get-CimInstance Win32_PnPEntity -ErrorAction Stop |
            Where-Object { $_.Name -match 'COM\d+' -and $_.Name -match 'CP210|CH340|FTDI|Silicon Labs|USB-SERIAL' } |
            ForEach-Object { if ($_.Name -match '(COM\d+)') { $matches[1] } } |
            Select-Object -First 1
    } catch { $null }
}

function NormalizePort([string]$p) {
    # Accept bare numbers: "11" -> "COM11"
    if ($p -match '^\d+$') { return "COM$p" }
    return $p
}

function SelectPort {
    $auto = GetEspPort
    if ($auto) {
        Write-Host "  Auto-detected: $auto" -ForegroundColor Green
        $ans = (Read-Host "  Use $auto? (Enter = yes  /  type COMx to override)").Trim()
        $script:Port = if ($ans -eq '') { $auto } else { NormalizePort $ans }
    } elseif ($script:Port) {
        Write-Host "  Last used: $($script:Port)" -ForegroundColor Yellow
        $ans = (Read-Host "  Use $($script:Port)? (Enter = yes  /  type COMx to override)").Trim()
        if ($ans -ne '') { $script:Port = NormalizePort $ans }
    } else {
        Write-Host '  No ESP32 detected on USB.' -ForegroundColor Yellow
        $raw = (Read-Host '  Enter COM port (e.g. COM5)').Trim()
        $script:Port = NormalizePort $raw
    }
    SaveConfig
    return ($script:Port -ne '')
}

# Run idf.py from firmware root. Output streams directly to console -- no capture.
function RunIdf {
    Push-Location $FwRoot
    try {
        & idf.py @args
        return $LASTEXITCODE
    } finally {
        Pop-Location
    }
}

function ShowResult([int]$ec, [datetime]$start) {
    $dur = [int]((Get-Date) - $start).TotalSeconds
    Write-Host ''
    Write-Host '  ------------------------------------------------' -ForegroundColor DarkGray
    if ($ec -eq 0) {
        Write-Host "  [OK]   Finished in ${dur}s" -ForegroundColor Green
    } else {
        Write-Host "  [FAIL] Failed (exit $ec) in ${dur}s" -ForegroundColor Red
    }
    Log "ec=$ec dur=${dur}s"
}

# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------

function Do-Build {
    Header 'Build'
    if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
        Write-Host '  idf.py not found. Close this window and open via ESP-IDF shortcut.' -ForegroundColor Red
        Pause; return
    }
    Write-Host "  Project : $FwRoot" -ForegroundColor Cyan
    Write-Host '  Command : idf.py build' -ForegroundColor DarkGray
    Write-Host ''
    Log 'build start'
    $t = Get-Date
    RunIdf build
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-FullClean {
    Header 'Full Clean + Rebuild'
    Write-Host '  Deletes build/ and rebuilds everything from scratch.' -ForegroundColor Yellow
    Write-Host '  First build takes several minutes.' -ForegroundColor DarkGray
    Write-Host ''
    $ans = Read-Host '  Type YES to confirm'
    if ($ans -cne 'YES') { Write-Host '  Cancelled.'; Pause; return }
    Write-Host ''
    Log 'fullclean+build start'
    $t = Get-Date
    RunIdf fullclean
    $ec1 = $LASTEXITCODE
    if ($ec1 -eq 0) {
        Write-Host ''
        Write-Host '  -- Starting build --' -ForegroundColor Cyan
        Write-Host ''
        RunIdf build
    }
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-Flash {
    Header 'Flash'
    if (-not (SelectPort)) { Write-Host '  No port. Cancelled.' -ForegroundColor Red; Pause; return }
    Write-Host "  Port    : $($script:Port)" -ForegroundColor Cyan
    Write-Host '  Command : idf.py flash' -ForegroundColor DarkGray
    Write-Host ''
    Log "flash $($script:Port)"
    $t = Get-Date
    RunIdf -p $script:Port flash
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-Monitor {
    Header 'Monitor'
    if (-not (SelectPort)) { Write-Host '  No port. Cancelled.' -ForegroundColor Red; Pause; return }
    Write-Host "  Port    : $($script:Port)  Baud: $($script:Baud)" -ForegroundColor Cyan
    Write-Host '  Press Ctrl+] to exit monitor.' -ForegroundColor DarkGray
    Write-Host ''
    Log "monitor $($script:Port)"
    Push-Location $FwRoot
    try { & idf.py -p $script:Port -b $script:Baud monitor }
    finally { Pop-Location }
    Pause
}

function Do-FlashMonitor {
    Header 'Flash + Monitor'
    if (-not (SelectPort)) { Write-Host '  No port. Cancelled.' -ForegroundColor Red; Pause; return }
    Write-Host "  Port    : $($script:Port)  Baud: $($script:Baud)" -ForegroundColor Cyan
    Write-Host '  Flashes firmware then opens monitor. Ctrl+] to exit.' -ForegroundColor DarkGray
    Write-Host ''
    Log "flash+monitor $($script:Port)"
    Push-Location $FwRoot
    try { & idf.py -p $script:Port -b $script:Baud flash monitor }
    finally { Pop-Location }
    Pause
}

function Do-Detect {
    Header 'Detect USB Devices'
    Write-Host '  Scanning...' -ForegroundColor DarkGray
    Write-Host ''
    try {
        $all  = @(Get-CimInstance Win32_PnPEntity -ErrorAction Stop | Where-Object { $_.Name -match 'COM\d+' })
        $esp  = @($all | Where-Object { $_.Name -match 'CP210|CH340|FTDI|Silicon Labs|USB-SERIAL' })
        $rest = @($all | Where-Object { $_.Name -notmatch 'CP210|CH340|FTDI|Silicon Labs|USB-SERIAL' })
    } catch { $all = @(); $esp = @(); $rest = @() }

    Write-Host '  ESP32-compatible adapters:' -ForegroundColor Cyan
    if ($esp.Count -gt 0) {
        $esp | ForEach-Object { Write-Host "    $($_.Name)" -ForegroundColor White }
    } else {
        Write-Host '    (none detected -- connect device or check driver)' -ForegroundColor Yellow
    }
    Write-Host ''
    Write-Host '  Other COM ports:' -ForegroundColor DarkGray
    if ($rest.Count -gt 0) {
        $rest | ForEach-Object { Write-Host "    $($_.Name)" -ForegroundColor DarkGray }
    } else {
        Write-Host '    (none)' -ForegroundColor DarkGray
    }
    Write-Host ''
    Write-Host '  Tip: CP210x = Silicon Labs  CH340 = common module  FTDI = FT232' -ForegroundColor DarkGray
    Pause
}

function Do-Diagnostics {
    Header 'Diagnostics'
    $p = 0; $w = 0; $f = 0

    function Pass([string]$lbl, [string]$note='') {
        Write-Host ("  PASS  {0,-30} {1}" -f $lbl, $note) -ForegroundColor Green;  $script:p++
    }
    function Warn([string]$lbl, [string]$note='') {
        Write-Host ("  WARN  {0,-30} {1}" -f $lbl, $note) -ForegroundColor Yellow; $script:w++
    }
    function Fail([string]$lbl, [string]$note='') {
        Write-Host ("  FAIL  {0,-30} {1}" -f $lbl, $note) -ForegroundColor Red;    $script:f++
    }

    # idf.py
    if (Get-Command idf.py -ErrorAction SilentlyContinue) { Pass 'idf.py' }
    else { Fail 'idf.py' 'not in PATH -- source ESP-IDF profile' }

    # Project files
    if (Test-Path (Join-Path $FwRoot 'CMakeLists.txt'))    { Pass 'CMakeLists.txt' }
    else { Fail 'CMakeLists.txt' 'wrong firmware root?' }

    if (Test-Path (Join-Path $FwRoot 'main\Kconfig.projbuild')) { Pass 'Kconfig.projbuild' }
    else { Fail 'Kconfig.projbuild' 'expected at main/Kconfig.projbuild' }

    $sdk = Join-Path $FwRoot 'sdkconfig'
    $def = Join-Path $FwRoot 'sdkconfig.defaults'
    if     (Test-Path $sdk) { Pass 'sdkconfig' }
    elseif (Test-Path $def) { Warn 'sdkconfig' 'missing -- will be created on first build' }
    else                    { Fail 'sdkconfig' 'no sdkconfig or sdkconfig.defaults' }

    # Binary
    $bin = Get-ChildItem $BuildDir -Filter 'vitalkare_firmware.bin' -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($bin) { Pass 'Firmware binary' ("{0} KB" -f [int]($bin.Length/1KB)) }
    else       { Warn 'Firmware binary' 'run build first' }

    # USB
    $port = GetEspPort
    if ($port) { Pass 'ESP32 on USB' $port }
    else        { Warn 'ESP32 on USB' 'no device detected' }

    # IDF env
    if ($env:IDF_PATH) { Pass 'IDF_PATH' $env:IDF_PATH }
    else                { Warn 'IDF_PATH' 'not set in this session' }

    # Disk
    try {
        $free = [math]::Round((Get-PSDrive C -ErrorAction Stop).Free / 1GB, 1)
        if ($free -lt 1) { Warn 'Disk (C:)' "$free GB free -- builds may fail" }
        else              { Pass 'Disk (C:)' "$free GB free" }
    } catch { Warn 'Disk (C:)' 'could not read' }

    Write-Host ''
    Write-Host "  PASS: $($script:p)  WARN: $($script:w)  FAIL: $($script:f)" -ForegroundColor White
    Pause
}

function Do-Erase {
    Header 'Erase Flash  -- DANGER'
    if (-not (SelectPort)) { Write-Host '  No port. Cancelled.' -ForegroundColor Red; Pause; return }
    Write-Host ''
    Write-Host '  +--------------------------------------------------+' -ForegroundColor Red
    Write-Host '  |  WARNING: ERASES ENTIRE FLASH (firmware + NVS)  |' -ForegroundColor Red
    Write-Host "  |  Target port: $($script:Port.PadRight(35))|" -ForegroundColor Red
    Write-Host '  +--------------------------------------------------+' -ForegroundColor Red
    Write-Host ''
    $ans = Read-Host '  Type YES to confirm'
    if ($ans -cne 'YES') { Write-Host '  Cancelled.'; Pause; return }
    Log "erase $($script:Port)"
    $t = Get-Date
    RunIdf -p $script:Port erase-flash
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-WifiConfig {
    Header 'Configure WiFi Credentials'

    $defaultsFile = Join-Path $FwRoot 'sdkconfig.defaults'
    if (-not (Test-Path $defaultsFile)) {
        Write-Host '  sdkconfig.defaults not found.' -ForegroundColor Red
        Pause; return
    }

    $content = Get-Content $defaultsFile -Raw

    # Read current values
    $curSsid = if ($content -match 'CONFIG_VK_WIFI_SSID="([^"]*)"')     { $matches[1] } else { '' }
    $hasPass  = if ($content -match 'CONFIG_VK_WIFI_PASSWORD="([^"]+)"') { '(definida)' } else { '(vazia)' }

    Write-Host "  SSID atual    : $curSsid" -ForegroundColor Cyan
    Write-Host "  Senha atual   : $hasPass"  -ForegroundColor Cyan
    Write-Host ''
    Write-Host '  Pressione Enter para manter o valor atual.' -ForegroundColor DarkGray
    Write-Host ''

    $newSsid = (Read-Host '  Novo SSID').Trim()
    $newPass  = (Read-Host '  Nova senha').Trim()

    if ($newSsid -eq '' -and $newPass -eq '') {
        Write-Host '  Nenhuma alteração.' -ForegroundColor Yellow
        Pause; return
    }

    if ($newSsid -ne '') {
        $content = $content -replace 'CONFIG_VK_WIFI_SSID="[^"]*"', "CONFIG_VK_WIFI_SSID=`"$newSsid`""
        Write-Host "  SSID atualizado: $newSsid" -ForegroundColor Green
    }
    # Always update password if typed (allows clearing by entering a space then trimming handles it)
    if ($newPass -ne '') {
        $content = $content -replace 'CONFIG_VK_WIFI_PASSWORD="[^"]*"', "CONFIG_VK_WIFI_PASSWORD=`"$newPass`""
        Write-Host '  Senha atualizada.' -ForegroundColor Green
    }

    [System.IO.File]::WriteAllText($defaultsFile, $content, [System.Text.Encoding]::ASCII)

    # Remove stale sdkconfig so it regenerates from defaults on next build
    $sdk = Join-Path $FwRoot 'sdkconfig'
    if (Test-Path $sdk) {
        Remove-Item $sdk -Force
        Write-Host '  sdkconfig removido — será regenerado no próximo build.' -ForegroundColor DarkGray
    }

    Write-Host ''
    Write-Host '  Credenciais salvas em sdkconfig.defaults.' -ForegroundColor Green
    Write-Host '  Execute Build (1) ou Flash (3) para gravar no dispositivo.' -ForegroundColor Cyan
    Log "wifi-config ssid=$newSsid"
    Pause
}

function Do-Commission {
    Header 'Comissionar Dispositivo'

    $defaultsFile = Join-Path $FwRoot 'sdkconfig.defaults'
    $sdkFile      = Join-Path $FwRoot 'sdkconfig'

    if (-not (Test-Path $defaultsFile)) {
        Write-Host '  sdkconfig.defaults nao encontrado.' -ForegroundColor Red
        Pause; return
    }

    $dc = [System.IO.File]::ReadAllText($defaultsFile, [System.Text.Encoding]::UTF8)

    $curId   = if ($dc -match 'CONFIG_VK_DEVICE_ID="([^"]*)"')         { $matches[1] } else { 'esp32-bed-001' }
    $curWard = if ($dc -match 'CONFIG_VK_DEVICE_WARD="([^"]*)"')       { $matches[1] } else { 'A' }
    $curBed  = if ($dc -match 'CONFIG_VK_DEVICE_BED_NUMBER="([^"]*)"') { $matches[1] } else { '01' }

    Write-Host "  ID atual    : $curId"   -ForegroundColor Cyan
    Write-Host "  Ala atual   : $curWard" -ForegroundColor Cyan
    Write-Host "  Leito atual : $curBed"  -ForegroundColor Cyan
    Write-Host ''
    Write-Host '  Deixe em branco para manter o valor atual.' -ForegroundColor DarkGray
    Write-Host ''

    $newId   = (Read-Host "  Novo ID        (Enter = manter '$curId')").Trim()
    $newWard = (Read-Host "  Nova Ala       (Enter = manter '$curWard')").Trim()
    $newBed  = (Read-Host "  Novo Leito     (Enter = manter '$curBed')").Trim()

    if ($newId -eq '' -and $newWard -eq '' -and $newBed -eq '') {
        Write-Host ''
        Write-Host '  Nenhuma alteracao.' -ForegroundColor Yellow
        Pause; return
    }

    $finalId   = if ($newId   -ne '') { $newId   } else { $curId   }
    $finalWard = if ($newWard -ne '') { $newWard } else { $curWard }
    $finalBed  = if ($newBed  -ne '') { $newBed  } else { $curBed  }

    # Updates a file in-place preserving LF line endings (required by ESP-IDF)
    function Apply-DeviceValues([string]$Path) {
        if (-not (Test-Path $Path)) { return $false }
        $c = [System.IO.File]::ReadAllText($Path, [System.Text.Encoding]::UTF8)
        $c = $c -replace '(CONFIG_VK_DEVICE_ID=)"[^"]*"',         "`$1`"$finalId`""
        $c = $c -replace '(CONFIG_VK_DEVICE_WARD=)"[^"]*"',       "`$1`"$finalWard`""
        $c = $c -replace '(CONFIG_VK_DEVICE_BED_NUMBER=)"[^"]*"', "`$1`"$finalBed`""
        [System.IO.File]::WriteAllText($Path, $c, [System.Text.Encoding]::UTF8)
        return $true
    }

    Write-Host ''
    if (Apply-DeviceValues $defaultsFile) {
        Write-Host '  [OK] sdkconfig.defaults atualizado' -ForegroundColor Green
    }
    if (Apply-DeviceValues $sdkFile) {
        Write-Host '  [OK] sdkconfig atualizado' -ForegroundColor Green
    }

    Write-Host ''
    Write-Host '  +------------------------------------+' -ForegroundColor Cyan
    Write-Host "  |  ID    : $($finalId.PadRight(27))|" -ForegroundColor White
    Write-Host "  |  Ala   : $($finalWard.PadRight(27))|" -ForegroundColor White
    Write-Host "  |  Leito : $($finalBed.PadRight(27))|" -ForegroundColor White
    Write-Host '  +------------------------------------+' -ForegroundColor Cyan
    Write-Host ''
    Write-Host '  Proximo passo: Build (1)  depois  Flash (3)' -ForegroundColor Yellow
    Log "commission id=$finalId ward=$finalWard bed=$finalBed"
    Pause
}

function Do-Menuconfig {
    Header 'Menuconfig'
    Write-Host '  Opening firmware Kconfig menu...' -ForegroundColor Cyan
    Write-Host '  Save: S    Quit: Q' -ForegroundColor DarkGray
    Write-Host ''
    Log 'menuconfig'
    RunIdf menuconfig | Out-Null
    Pause
}

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

LoadConfig
Log '=== Tool started ==='

$running = $true
while ($running) {
    Header
    StatusLine

    Write-Host '  BUILD' -ForegroundColor White
    Write-Host '   1  Build'
    Write-Host '   2  Full clean + rebuild'
    Write-Host ''
    Write-Host '  FLASH / MONITOR' -ForegroundColor White
    Write-Host '   3  Flash'
    Write-Host '   4  Monitor  (serial output)'
    Write-Host '   5  Flash + Monitor'
    Write-Host ''
    Write-Host '  TOOLS' -ForegroundColor White
    Write-Host '   6  Detect ESP32 on USB'
    Write-Host '   7  Diagnostics'
    Write-Host '   8  Configure WiFi credentials' -ForegroundColor Cyan
    Write-Host '   9  Comissionar dispositivo  (ID / Ala / Leito)' -ForegroundColor Cyan
    Write-Host '  10  Menuconfig  (firmware settings)'
    Write-Host '  11  Erase flash  (DANGER)' -ForegroundColor Yellow
    Write-Host ''
    Write-Host '   0  Exit' -ForegroundColor Red
    Write-Host ''

    $opt = (Read-Host '  Select').Trim()

    try {
        switch ($opt) {
            '1' { Do-Build }
            '2' { Do-FullClean }
            '3' { Do-Flash }
            '4' { Do-Monitor }
            '5' { Do-FlashMonitor }
            '6'  { Do-Detect }
            '7'  { Do-Diagnostics }
            '8'  { Do-WifiConfig }
            '9'  { Do-Commission }
            '10' { Do-Menuconfig }
            '11' { Do-Erase }
            '0' { $running = $false }
            ''  { }
            default {
                Write-Host "  Unknown option: $opt" -ForegroundColor Yellow
                Start-Sleep -Milliseconds 600
            }
        }
    } catch {
        Write-Host "  Error: $($_.Exception.Message)" -ForegroundColor Red
        Log "ERROR: $($_.Exception.Message)"
        Pause
    }
}

Log '=== Tool exited ==='
Write-Host '  Bye.' -ForegroundColor Cyan
Write-Host ''
