# NIDAR: YOLO on the Gazebo camera, detections to the Pi (2026-11-09).
# Started by tools/nidar_run.sh (WSL) in its own window; can also be run by hand:
#   powershell -ExecutionPolicy Bypass -File D:\NIDAR_DRONE_PROJECT\tools\nidar_yolo.ps1 -PiHost 10.218.70.147
param([string]$PiHost = "10.218.70.147")
$Host.UI.RawUI.WindowTitle = "NIDAR YOLO  (Gazebo camera -> Pi $PiHost)"
Set-Location -Path (Split-Path -Parent $PSScriptRoot)
$env:NIDAR_CAMERA_SOURCE = "gazebo"
$env:NIDAR_PI_HOST = $PiHost
py -3 nidar_v11_windows.py
