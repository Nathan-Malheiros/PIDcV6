#Requires -Version 5.1
# Ball Balancer 3RPS - Firmware Tool
# Lancado por bb_tool.bat, que carrega o profile do ESP-IDF antes.

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Raiz do firmware = pasta deste script
$FwRoot   = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $FwRoot 'build'
$BinName  = 'ball_balancer.bin'
$Target   = 'esp32s3'
$CfgFile  = Join-Path $FwRoot '.bb_tool_cfg'

# Fixa o alvo na sessao -> evita "'esp32' in the environment, 'esp32s3' in CMakeCache".
$env:IDF_TARGET = $Target

$script:Port = 'COM8'
$script:Baud = 115200

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

function Header([string]$title = '') {
    Clear-Host
    Write-Host ''
    Write-Host '  ===================================================' -ForegroundColor Magenta
    Write-Host '   Ball Balancer 3RPS  --  Firmware Tool  (ESP32-S3)' -ForegroundColor White
    if ($title) { Write-Host "   $title" -ForegroundColor Cyan }
    Write-Host '  ===================================================' -ForegroundColor Magenta
    Write-Host ''
}

function StatusLine {
    $idfOk = [bool](Get-Command idf.py -ErrorAction SilentlyContinue)
    Write-Host "  Root   : $FwRoot" -ForegroundColor Cyan
    Write-Host "  Target : $Target" -ForegroundColor Cyan
    Write-Host "  COM    : $($script:Port)   Baud: $($script:Baud)" -ForegroundColor Cyan
    if ($idfOk) { Write-Host '  IDF    : disponivel' -ForegroundColor Green }
    else        { Write-Host '  IDF    : NAO ENCONTRADO -- abra pelo atalho ESP-IDF / bb_tool.bat' -ForegroundColor Red }
    Write-Host ''
}

function Pause {
    Write-Host ''
    Write-Host '  Pressione Enter para voltar ao menu...' -ForegroundColor DarkGray
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

function NormalizePort([string]$p) {
    if ($p -match '^\d+$') { return "COM$p" }   # "8" -> "COM8"
    return $p
}

function SelectPort {
    Write-Host "  Porta atual: $($script:Port)" -ForegroundColor Yellow
    $ans = (Read-Host "  Enter = usar $($script:Port)  /  ou digite a porta (ex: COM8)").Trim()
    if ($ans -ne '') { $script:Port = NormalizePort $ans }
    SaveConfig
    return ($script:Port -ne '')
}

# Roda idf.py a partir da raiz do firmware (saida direto no console).
function RunIdf {
    Push-Location $FwRoot
    try { & idf.py @args; return $LASTEXITCODE }
    finally { Pop-Location }
}

function ShowResult([int]$ec, [datetime]$start) {
    $dur = [int]((Get-Date) - $start).TotalSeconds
    Write-Host ''
    Write-Host '  ---------------------------------------------------' -ForegroundColor DarkGray
    if ($ec -eq 0) { Write-Host "  [OK]   Concluido em ${dur}s" -ForegroundColor Green }
    else           { Write-Host "  [FALHA] Saida $ec em ${dur}s" -ForegroundColor Red }
}

function Require-Idf {
    if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
        Write-Host '  idf.py nao encontrado. Feche e abra de novo pelo bb_tool.bat' -ForegroundColor Red
        Pause; return $false
    }
    return $true
}

# ---------------------------------------------------------------------------
# Acoes
# ---------------------------------------------------------------------------

function Do-Build {
    Header 'Build'
    if (-not (Require-Idf)) { return }
    Write-Host '  Comando : idf.py build' -ForegroundColor DarkGray
    Write-Host ''
    $t = Get-Date
    RunIdf build
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-FullClean {
    Header 'Full clean + rebuild'
    if (-not (Require-Idf)) { return }
    Write-Host '  Apaga build/ e reconstroi do zero (resolve mismatch de target).' -ForegroundColor Yellow
    Write-Host '  A primeira build leva alguns minutos.' -ForegroundColor DarkGray
    Write-Host ''
    $ans = Read-Host '  Digite SIM para confirmar'
    if ($ans -cne 'SIM') { Write-Host '  Cancelado.'; Pause; return }
    $t = Get-Date
    RunIdf fullclean
    if ($LASTEXITCODE -eq 0) {
        Write-Host ''
        Write-Host '  -- set-target esp32s3 --' -ForegroundColor Cyan
        RunIdf set-target $Target
    }
    if ($LASTEXITCODE -eq 0) {
        Write-Host ''
        Write-Host '  -- build --' -ForegroundColor Cyan
        RunIdf build
    }
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-SetTarget {
    Header 'Set target (esp32s3)'
    if (-not (Require-Idf)) { return }
    Write-Host '  Regenera o sdkconfig e fixa o alvo em esp32s3.' -ForegroundColor DarkGray
    Write-Host ''
    $t = Get-Date
    RunIdf set-target $Target
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-Flash {
    Header 'Flash (UART/esptool)'
    if (-not (Require-Idf)) { return }
    if (-not (SelectPort)) { Write-Host '  Sem porta. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host "  Comando : idf.py -p $($script:Port) flash" -ForegroundColor DarkGray
    Write-Host ''
    $t = Get-Date
    RunIdf -p $script:Port flash
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-Monitor {
    Header 'Monitor (serial)'
    if (-not (Require-Idf)) { return }
    if (-not (SelectPort)) { Write-Host '  Sem porta. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host "  Porta : $($script:Port)  Baud: $($script:Baud)" -ForegroundColor Cyan
    Write-Host '  Saia do monitor com Ctrl+]' -ForegroundColor DarkGray
    Write-Host '  Comandos no firmware: SHOW | HIDDEN | PID | CAL | ?' -ForegroundColor DarkGray
    Write-Host ''
    Push-Location $FwRoot
    try { & idf.py -p $script:Port -b $script:Baud monitor }
    finally { Pop-Location }
    Pause
}

function Do-FlashMonitor {
    Header 'Flash + Monitor'
    if (-not (Require-Idf)) { return }
    if (-not (SelectPort)) { Write-Host '  Sem porta. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host "  Porta : $($script:Port)  Baud: $($script:Baud)" -ForegroundColor Cyan
    Write-Host '  Grava e abre o monitor. Ctrl+] para sair.' -ForegroundColor DarkGray
    Write-Host '  Comandos no firmware: SHOW | HIDDEN | PID | CAL | ?' -ForegroundColor DarkGray
    Write-Host ''
    Push-Location $FwRoot
    try { & idf.py -p $script:Port -b $script:Baud flash monitor }
    finally { Pop-Location }
    Pause
}

function Do-BuildFlashMonitor {
    Header 'Build + Flash + Monitor'
    if (-not (Require-Idf)) { return }
    if (-not (SelectPort)) { Write-Host '  Sem porta. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host "  Porta : $($script:Port)  Baud: $($script:Baud)" -ForegroundColor Cyan
    Write-Host '  Comandos no firmware: SHOW | HIDDEN | PID | CAL | ?' -ForegroundColor DarkGray
    Write-Host ''
    Push-Location $FwRoot
    try { & idf.py -p $script:Port -b $script:Baud build flash monitor }
    finally { Pop-Location }
    Pause
}

function Do-Erase {
    Header 'Erase flash  -- PERIGO'
    if (-not (Require-Idf)) { return }
    if (-not (SelectPort)) { Write-Host '  Sem porta. Cancelado.' -ForegroundColor Red; Pause; return }
    Write-Host ''
    Write-Host '  +-----------------------------------------------+' -ForegroundColor Red
    Write-Host '  |  APAGA TODA A FLASH (firmware + NVS/calib.)   |' -ForegroundColor Red
    Write-Host "  |  Porta: $($script:Port.PadRight(38))|" -ForegroundColor Red
    Write-Host '  +-----------------------------------------------+' -ForegroundColor Red
    Write-Host ''
    $ans = Read-Host '  Digite APAGAR para confirmar'
    if ($ans -cne 'APAGAR') { Write-Host '  Cancelado.'; Pause; return }
    $t = Get-Date
    RunIdf -p $script:Port erase-flash
    ShowResult $LASTEXITCODE $t
    Pause
}

function Do-Menuconfig {
    Header 'Menuconfig'
    if (-not (Require-Idf)) { return }
    Write-Host '  Salvar: S    Sair: Q' -ForegroundColor DarkGray
    Write-Host ''
    RunIdf menuconfig | Out-Null
    Pause
}

function Do-Detect {
    Header 'Detectar portas COM'
    Write-Host '  Procurando...' -ForegroundColor DarkGray
    Write-Host ''
    try {
        $all = @(Get-CimInstance Win32_PnPEntity -ErrorAction Stop |
                 Where-Object { $_.Name -match 'COM\d+' })
    } catch { $all = @() }
    if ($all.Count -gt 0) {
        $all | ForEach-Object { Write-Host "    $($_.Name)" -ForegroundColor White }
    } else {
        Write-Host '    (nenhuma porta COM encontrada)' -ForegroundColor Yellow
    }
    Write-Host ''
    Write-Host '  Dica: a placa S3 nativa aparece como "USB JTAG/serial" ou "USB Serial Device".' -ForegroundColor DarkGray
    Pause
}

function Do-Diagnostics {
    Header 'Diagnostico'
    $script:p = 0; $script:w = 0; $script:f = 0
    function Pass([string]$l,[string]$n=''){ Write-Host ("  PASS  {0,-26} {1}" -f $l,$n) -ForegroundColor Green;  $script:p++ }
    function Warn([string]$l,[string]$n=''){ Write-Host ("  WARN  {0,-26} {1}" -f $l,$n) -ForegroundColor Yellow; $script:w++ }
    function Fail([string]$l,[string]$n=''){ Write-Host ("  FAIL  {0,-26} {1}" -f $l,$n) -ForegroundColor Red;    $script:f++ }

    if (Get-Command idf.py -ErrorAction SilentlyContinue) { Pass 'idf.py' } else { Fail 'idf.py' 'fora do PATH' }
    if (Test-Path (Join-Path $FwRoot 'CMakeLists.txt'))    { Pass 'CMakeLists.txt' } else { Fail 'CMakeLists.txt' 'raiz errada?' }
    $sdk = Join-Path $FwRoot 'sdkconfig'
    $def = Join-Path $FwRoot 'sdkconfig.defaults'
    if     (Test-Path $sdk) { Pass 'sdkconfig' }
    elseif (Test-Path $def) { Warn 'sdkconfig' 'ausente -- criado no proximo build' }
    else                    { Fail 'sdkconfig' 'sem sdkconfig/defaults' }
    $bin = Get-ChildItem $BuildDir -Filter $BinName -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($bin) { Pass 'Binario' ("{0} KB" -f [int]($bin.Length/1KB)) } else { Warn 'Binario' 'rode o build' }
    if ($env:IDF_PATH)   { Pass 'IDF_PATH' $env:IDF_PATH } else { Warn 'IDF_PATH' 'nao setado' }
    if ($env:IDF_TARGET) { Pass 'IDF_TARGET' $env:IDF_TARGET } else { Warn 'IDF_TARGET' 'nao setado' }
    Write-Host ''
    Write-Host "  PASS: $($script:p)  WARN: $($script:w)  FAIL: $($script:f)" -ForegroundColor White
    Pause
}

# ---------------------------------------------------------------------------
# Loop principal
# ---------------------------------------------------------------------------

LoadConfig

$running = $true
while ($running) {
    Header
    StatusLine

    Write-Host '  BUILD' -ForegroundColor White
    Write-Host '   1  Build'
    Write-Host '   2  Full clean + set-target + rebuild'
    Write-Host '   3  Set target esp32s3 (regenera sdkconfig)'
    Write-Host ''
    Write-Host '  FLASH / MONITOR' -ForegroundColor White
    Write-Host '   4  Flash'
    Write-Host '   5  Monitor'
    Write-Host '   6  Flash + Monitor'
    Write-Host '   7  Build + Flash + Monitor' -ForegroundColor Cyan
    Write-Host ''
    Write-Host '  FERRAMENTAS' -ForegroundColor White
    Write-Host '   8  Detectar portas COM'
    Write-Host '   9  Diagnostico'
    Write-Host '  10  Menuconfig'
    Write-Host '  11  Erase flash  (PERIGO)' -ForegroundColor Yellow
    Write-Host ''
    Write-Host '   0  Sair' -ForegroundColor Red
    Write-Host ''

    $opt = (Read-Host '  Opcao').Trim()
    try {
        switch ($opt) {
            '1'  { Do-Build }
            '2'  { Do-FullClean }
            '3'  { Do-SetTarget }
            '4'  { Do-Flash }
            '5'  { Do-Monitor }
            '6'  { Do-FlashMonitor }
            '7'  { Do-BuildFlashMonitor }
            '8'  { Do-Detect }
            '9'  { Do-Diagnostics }
            '10' { Do-Menuconfig }
            '11' { Do-Erase }
            '0'  { $running = $false }
            ''   { }
            default { Write-Host "  Opcao invalida: $opt" -ForegroundColor Yellow; Start-Sleep -Milliseconds 600 }
        }
    } catch {
        Write-Host "  Erro: $($_.Exception.Message)" -ForegroundColor Red
        Pause
    }
}

Write-Host '  Ate mais.' -ForegroundColor Cyan
Write-Host ''
