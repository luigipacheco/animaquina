# tools/ursim-start.ps1
# Start the URSim PolyScope X simulator with a given robot model.
#
# Usage:
#   .\tools\ursim-start.ps1                  # default UR10e
#   .\tools\ursim-start.ps1 -RobotModel UR5e
#   .\tools\ursim-start.ps1 -RobotModel UR20
#   .\tools\ursim-start.ps1 -Stop            # stop and remove the container
#
# Valid models: UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e, UR20, UR30
# Once running, open http://localhost in your browser.
# Robot IP for animaquina: 127.0.0.1
# ExternalControl node host IP (inside URSim): 172.17.0.1  port 50002

param(
    [string]$RobotModel = "UR10e",
    [switch]$Stop
)

$ContainerName = "ursim-polyscopex"
$Image         = "universalrobots/ursim_polyscopex"

$ValidModels = @("UR3","UR3e","UR5","UR5e","UR10","UR10e","UR16e","UR20","UR30")

function Stop-Sim {
    $existing = docker ps -aq --filter "name=$ContainerName" 2>$null
    if ($existing) {
        Write-Host "Stopping $ContainerName ..."
        docker rm -f $ContainerName | Out-Null
        Write-Host "Done."
    } else {
        Write-Host "No running simulator found."
    }
}

if ($Stop) {
    Stop-Sim
    exit 0
}

if ($ValidModels -notcontains $RobotModel) {
    Write-Error "Unknown robot model '$RobotModel'. Valid: $($ValidModels -join ', ')"
    exit 1
}

# Ensure Docker is running
$null = docker ps 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Docker is not running. Starting Docker Desktop ..."
    Start-Process "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    Write-Host "Waiting for Docker to be ready ..."
    $waited = 0
    while ($waited -lt 120) {
        Start-Sleep -Seconds 5; $waited += 5
        $null = docker ps 2>$null
        if ($LASTEXITCODE -eq 0) { Write-Host "Docker ready."; break }
        Write-Host "  ... $waited`s"
    }
    if ($LASTEXITCODE -ne 0) { Write-Error "Docker did not start in time."; exit 1 }
}

Stop-Sim

Write-Host "Starting URSim PolyScope X as $RobotModel ..."

# ROBOT_TYPE is read by setup.sh inside the container before .env is written,
# so passing it here ensures the correct model from the very first boot.
docker run -d --name $ContainerName --privileged `
    -e ROBOT_TYPE=$RobotModel `
    -p 80:80 -p 443:443 -p 5900:5900 `
    -p 29999:29999 `
    -p 30001:30001 -p 30002:30002 -p 30003:30003 -p 30004:30004 `
    -p 50002:50002 `
    $Image | Out-Null

Write-Host "Waiting for simulator to boot (this takes ~60s) ..."
$waited = 0
$ready = $false
while ($waited -lt 150) {
    Start-Sleep -Seconds 6; $waited += 6
    # Poll the controller's robot-state REST API (served inside the container).
    # POWER_OFF means the controller has fully attached and is ready for the
    # one-time safety confirmation in the UI.
    $state = docker exec $ContainerName sh -c "curl -s 'http://172.19.0.8:30015/rest-api/robot/state' 2>/dev/null" 2>$null
    if ($state -match '"robotMode":"([A-Z_]+)".*"safetyMode":"([A-Z_]+)"') {
        $rm = $matches[1]; $sm = $matches[2]
        Write-Host "  [$waited`s] robotMode=$rm  safetyMode=$sm"
        if ($rm -eq "POWER_OFF" -or $rm -eq "IDLE" -or $rm -eq "RUNNING") { $ready = $true; break }
    } else {
        Write-Host "  ... booting $waited`s"
    }
}
if (-not $ready) {
    Write-Host "  (Controller did not report ready in time — check 'docker logs $ContainerName')"
}

# NOTE: Do NOT delete safety.conf / backend/safety files. The controller needs
# a valid, CRC-checked safety config to compare against; removing it triggers a
# persistent POLYSCOPE_SAFETY_MISMATCH fault (User parameters: BAD). A clean
# container boots with matching safety params for the chosen model on its own.
# On first boot you confirm the safety parameters once in the browser UI
# (click the fault banner -> Settings > Safety -> Apply, blank password).

Write-Host ""
Write-Host "URSim PolyScope X is ready as $RobotModel."
Write-Host "  Browser UI        : http://localhost"
Write-Host "  Robot IP          : 127.0.0.1  (use this in animaquina)"
Write-Host "  ExtControl host   : 172.17.0.1  port 50002  (set in the External Control node)"
Write-Host ""
Write-Host "First-boot steps in the browser (one time per fresh container):"
Write-Host "  1. Confirm the safety configuration if prompted (blank password)."
Write-Host "  2. Power on the robot, then Start to release the brakes."
Write-Host ""
Write-Host "To stop: .\tools\ursim-start.ps1 -Stop"
