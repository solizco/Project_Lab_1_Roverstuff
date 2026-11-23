param(
    [string]$Target = "projectlab_rover"
)
 
Write-Host "Building..."
cmake --build build -j4
if ($LASTEXITCODE -ne 0) { Write-Host "Build failed!"; exit 1 }
 
Write-Host "Build OK. Hold BOOTSEL and replug USB, then press Enter..."
Read-Host
 
$UF2 = "build\$Target.uf2"
 
if (-not (Test-Path $UF2)) {
    Write-Host "UF2 not found: $UF2"
    Write-Host "Valid targets: projectlab_rover, phase1b_blob"
    exit 1
}
 
$DRIVE = (Get-PSDrive | Where-Object { $_.Description -like "*RP2350*" -or $_.Name -eq "D" } | Select-Object -First 1).Root
 
if ($DRIVE) {
    Write-Host "Flashing $Target to $DRIVE..."
    Copy-Item $UF2 $DRIVE
    Write-Host "Done! Pico is rebooting."
} else {
    Write-Host "Pico not found. Make sure BOOTSEL is held while plugging in."
}
 