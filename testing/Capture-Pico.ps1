[CmdletBinding()]
param(
    [Parameter(Mandatory)][ValidateSet('bench','course')][string]$Image,
    [Parameter(Mandatory)][string]$Tester,
    [ValidatePattern('^COM[0-9]+$')][string]$Port,
    [ValidateRange(0,86400)][int]$Seconds = 0
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$suffix = if ($Image -eq 'bench') { '_test' } else { '' }
$uf2 = Join-Path $root "build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt$suffix.uf2"
$build = Get-Content -Raw -LiteralPath "$uf2.info" | ConvertFrom-StringData
if ((Get-FileHash -LiteralPath $uf2).Hash -ne $build.sha256 -or $Image -ne $build.profile) {
    throw 'UF2 and build record differ. Rebuild with testing/run-demo.sh.'
}
if (!$Port) {
    $ports = @([System.IO.Ports.SerialPort]::GetPortNames())
    if ($ports.Count -ne 1) { throw "Specify -Port COMn. Available ports: $($ports -join ', ')" }
    $Port = $ports[0]
}
$stamp = Get-Date -Format 'yyyy-MM-dd_HHmmss_fff'
$path = Join-Path $PSScriptRoot "system/logs/${stamp}_${Image}_$($build.commit.Substring(0,7)).txt"
$file = [IO.File]::Open($path, 'CreateNew', 'Write', 'Read')
$writer = [IO.StreamWriter]::new($file)
$serial = [System.IO.Ports.SerialPort]::new($Port,115200)
$serial.DtrEnable = $true
$serial.ReadTimeout = 300
$started = $null
try {
    $writer.AutoFlush = $true
    $writer.WriteLine("Started $(Get-Date -Format o) | Tester $Tester | $Port")
    $writer.WriteLine("Selected UF2 $([IO.Path]::GetFileName($uf2)) | SHA256 $($build.sha256)")
    $writer.WriteLine("Build $($build.commit) | tracked firmware edits $($build.tracked_firmware_edits)")
    Write-Host "Saving $path. Flash the selected UF2 or reset the Pico. Ctrl+C finishes."
    while (!$started -or !$Seconds -or ((Get-Date) - $started).TotalSeconds -lt $Seconds) {
        if (!$serial.IsOpen) {
            if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $Port) { Start-Sleep -Milliseconds 100; continue }
            try { $serial.Open() } catch [System.IO.IOException] { Start-Sleep -Milliseconds 100; continue }
        }
        try {
            $chunk = $serial.ReadExisting()
            if ($chunk) {
                if (!$started) { $started = Get-Date }
                $writer.Write($chunk)
                [Console]::Write($chunk)
            } else { Start-Sleep -Milliseconds 50 }
        } catch [System.IO.IOException] { $serial.Close() }
    }
} finally {
    $serial.Dispose()
    $writer.Dispose()
    Write-Host "Saved $path"
}

