# Configuration
$COM_PORT = "COM3"
$PLATFORM = "octopus_pro"

# Check if COM port is available
Write-Host "Checking COM port $COM_PORT availability..."
try {
    $port = New-Object System.IO.Ports.SerialPort $COM_PORT
    $port.Open()
    $port.Close()
    $port.Dispose()
    Write-Host "COM port $COM_PORT is available" -ForegroundColor Green
} catch {
    Write-Host "ERROR: COM port $COM_PORT is in use by another device or not available" -ForegroundColor Red
    Write-Host "Please close any programs using the serial port (e.g., serial monitors, other terminals)" -ForegroundColor Yellow
    Write-Host "Then run this script again." -ForegroundColor Yellow
    exit 1
}

# Run tests
Write-Host "Running tests on $COM_PORT..."
west twister --device-testing --device-serial=$COM_PORT --platform=$PLATFORM --testsuite-root=./tests --board-root=./ --clobber-output --inline-logs
