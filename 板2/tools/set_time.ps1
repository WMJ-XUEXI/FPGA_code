param(
    [string]$Port,
    [string]$At,
    [switch]$ListPorts
)
$ErrorActionPreference = 'Stop'
$portDevices = @()
try {
    $portDevices = @(Get-PnpDevice -Class Ports -PresentOnly -ErrorAction Stop)
} catch {
    # Device metadata is optional; basic port enumeration still works.
}
if ($ListPorts) {
    if ($portDevices.Count -gt 0) {
        $portDevices | Select-Object Status, FriendlyName | Format-Table -AutoSize
    } else {
        [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
        Write-Warning 'Device names unavailable. Check Device Manager; choose the board USB serial port, not a Bluetooth port.'
    }
    return
}
if ([string]::IsNullOrWhiteSpace($Port)) {
    throw 'Specify -Port COMx. Use -ListPorts to list serial ports.'
}
$Port = $Port.Trim().ToUpperInvariant()
if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $Port) {
    throw "Serial port $Port does not exist. Connect the board USB cable and use -ListPorts."
}
$selectedDevice = $portDevices | Where-Object { $_.FriendlyName -match ('\(' + [regex]::Escape($Port) + '\)$') } | Select-Object -First 1
if ($selectedDevice -and $selectedDevice.InstanceId -like 'BTHENUM\*') {
    throw "$Port is a Bluetooth serial port, not the FPGA USB serial port. Use -ListPorts and select USB Serial Port (COMx)."
}
if ($selectedDevice) { Write-Output ('Using ' + $selectedDevice.FriendlyName) }
if ($At) {
    $clockSetting = [DateTime]::ParseExact($At, 'yyyyMMddHHmmss', [Globalization.CultureInfo]::InvariantCulture)
    if ($clockSetting.Year -lt 2000) { throw 'Year must be 2000 or later.' }
}
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 2000
$serial.WriteTimeout = 2000
try {
    $serial.Open()
    Start-Sleep -Milliseconds 200
    $serial.DiscardInBuffer()
    if (-not $At) { $clockSetting = Get-Date }
    $payload = $clockSetting.ToString('yyyyMMddHHmmss', [Globalization.CultureInfo]::InvariantCulture)
    try {
        $serial.Write("T$payload`n")
    } catch {
        throw "Cannot write to $Port. Check that this is the board USB serial port and reconnect its USB cable. Driver error: $($_.Exception.Message)"
    }
    try {
        $ack = $serial.ReadChar()
    } catch {
        throw "Command sent to $Port, but no FPGA reply. Ensure sc520_calendar.bit is loaded and the board is powered. Driver error: $($_.Exception.Message)"
    }
    if ($ack -ne [int][char]'K') { throw "FPGA rejected calibration (reply $ack)." }
    Write-Output ('FPGA time set: ' + $clockSetting.ToString('yyyy-MM-dd HH:mm:ss') + ' (computer local time; no time-zone conversion)')
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
