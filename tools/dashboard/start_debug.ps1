# start_debug.ps1 - one-click Wi-Fi debugging for the CSC28 PicoCar.
#
# Starts Mosquitto with C:\mosq\car.conf (MQTT 1883 + WebSocket 9001),
# starts car_link.py (broker beacon + dashboard on :8000) and opens
# http://localhost:8000. Run it from anywhere, or double-click
# start_debug.bat next to it. Close the two windows it opens to stop.
#
# Owner: Buddy 1 (communications). Laptop tool, not car firmware.

$ErrorActionPreference = 'Stop'
$here     = Split-Path -Parent $MyInvocation.MyCommand.Path
$confDir  = 'C:\mosq'
$conf     = Join-Path $confDir 'car.conf'
$pageUrl  = 'http://localhost:8000'

function Say($text) { Write-Host "start_debug: $text" }

# ---- 1. Mosquitto ----------------------------------------------------
$mosq = $null
$cmd  = Get-Command mosquitto -ErrorAction SilentlyContinue
if ($cmd) { $mosq = $cmd.Source }
if (-not $mosq) {
    $candidates = @(
        (Join-Path $env:ProgramFiles 'mosquitto\mosquitto.exe'),
        (Join-Path ${env:ProgramFiles(x86)} 'mosquitto\mosquitto.exe'),
        (Join-Path $confDir 'mosquitto.exe')
    )
    foreach ($c in $candidates) {
        if ($c -and (Test-Path $c)) { $mosq = $c; break }
    }
}
if (-not $mosq) {
    Say 'mosquitto.exe not found. Install it once with:'
    Say '    winget install EclipseFoundation.Mosquitto'
    Say '(or the Windows 64-bit installer from mosquitto.org), then run this again.'
    exit 1
}
Say "Mosquitto: $mosq"

if (-not (Test-Path $conf)) {
    New-Item -ItemType Directory -Force $confDir | Out-Null
    Set-Content -Path $conf -Encoding ASCII -Value @(
        'listener 1883 0.0.0.0',
        'listener 9001 0.0.0.0',
        'protocol websockets',
        'allow_anonymous true'
    )
    Say "created $conf"
}

# The installer's Windows service grabs port 1883 with the default config
# (no WebSocket listener): stop it, and any old mosquitto window.
$svc = Get-Service mosquitto -ErrorAction SilentlyContinue
if ($svc -and ($svc.Status -eq 'Running')) {
    try {
        Stop-Service mosquitto -ErrorAction Stop
        Say 'stopped the mosquitto Windows service'
    } catch {
        Say 'the mosquitto service is running and needs an administrator to stop it:'
        Say '    right-click start_debug.bat > Run as administrator (once), or'
        Say '    Set-Service mosquitto -StartupType Manual   (admin PowerShell, once)'
        exit 1
    }
}
Get-Process mosquitto -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Milliseconds 500

Start-Process -FilePath $mosq -ArgumentList @('-c', "`"$conf`"", '-v') `
    -WindowStyle Normal
Say 'Mosquitto started in its own window (look for port 1883 and 9001 lines)'

# The car connects in to TCP 1883; a new hotspot is a Public network.
$rule = Get-NetFirewallRule -ErrorAction SilentlyContinue |
    Where-Object { $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' } |
    Get-NetFirewallPortFilter -ErrorAction SilentlyContinue |
    Where-Object { $_.LocalPort -eq '1883' }
if (-not $rule) {
    Say 'WARNING: no inbound firewall rule for TCP 1883 found. If the car prints'
    Say '"mqtt: connection status 256", run once in an administrator PowerShell:'
    Say '    New-NetFirewallRule -DisplayName "Mosquitto MQTT 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any'
}

# ---- 2. car_link.py (beacon + dashboard) -----------------------------
$py = Get-Command py -ErrorAction SilentlyContinue
$pyArgs = @()
if ($py) { $pyArgs = @('-3') } else { $py = Get-Command python -ErrorAction SilentlyContinue }
if (-not $py) {
    Say 'Python not found (py or python). Install Python 3 and run this again.'
    exit 1
}
Get-CimInstance Win32_Process -Filter "Name like 'py%'" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -like '*car_link.py*' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }

$link = Join-Path $here 'car_link.py'
Start-Process -FilePath $py.Source -ArgumentList ($pyArgs + @("`"$link`"")) `
    -WorkingDirectory $here -WindowStyle Normal
Say 'car_link.py started in its own window (broker beacon every second)'

# ---- 3. Dashboard ----------------------------------------------------
Start-Sleep -Seconds 2
Start-Process $pageUrl
Say "opened $pageUrl - broker localhost, port 9001, Connect (remembered next time)"
Say 'now power the car; press RUN only when the Car pill shows ONLINE'
