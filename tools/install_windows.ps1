# =====================================================================
# NIDAR AirMouse - Windows part of a new laptop's setup (2026-11-11).
# Open PowerShell AS ADMINISTRATOR (Start menu -> type PowerShell -> right-click -> Run as administrator), then:
#
#   irm https://raw.githubusercontent.com/abhi-gupta2005/NIDAR_DRONE_PROJECT/main/tools/install_windows.ps1 | iex
#
# (or, when the project is already on this laptop:
#   powershell -ExecutionPolicy Bypass -File D:\NIDAR_DRONE_PROJECT\tools\install_windows.ps1 )
#
# It does, skipping whatever is already there:
#   1. Git for Windows, then the project from GitHub into D:\NIDAR_DRONE_PROJECT (C:\ if there is no D: drive)
#   2. Python 3.12 + the camera AI packages (ultralytics YOLO, OpenCV, numpy) for nidar_v11_windows.py
#   3. WSL 2 + Ubuntu 24.04 (Windows asks for ONE restart the first time)
# and then prints the one command to type in Ubuntu (tools/install_wsl.sh installs ROS 2, Gazebo, PX4, ...).
# =====================================================================

$RepoUrl = "https://github.com/abhi-gupta2005/NIDAR_DRONE_PROJECT.git"

function Say($m) { Write-Host "[NIDAR] $m" -ForegroundColor Cyan }
function Warn($m) { Write-Host "[NIDAR] $m" -ForegroundColor Yellow }
function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
                [Environment]::GetEnvironmentVariable("Path", "User")
}
function Has($cmd) { return [bool](Get-Command $cmd -ErrorAction SilentlyContinue) }
function Winget-Install($id) {
    if (-not (Has "winget")) {
        Warn "winget is missing: install 'App Installer' from the Microsoft Store, then run this again."
        throw "winget missing"
    }
    winget install -e --id $id --silent --accept-package-agreements --accept-source-agreements
    Refresh-Path
}

function Install-Nidar {
    $admin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
             ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
    if (-not $admin) {
        Warn "Please run PowerShell as administrator (right-click -> Run as administrator) and try again."
        return
    }

    # ---------------------------------------------------------- 1. the project
    Say "1/3 the project"
    if ($PSScriptRoot -and (Test-Path (Join-Path $PSScriptRoot "nidar_run.sh"))) {
        $Proj = Split-Path -Parent $PSScriptRoot
        Say "    using $Proj"
    } else {
        if ($RepoUrl -like "__*") { Warn "This copy has no GitHub address (run tools/publish_github.sh first)."; return }
        if (-not (Has "git")) { Say "    installing Git for Windows"; Winget-Install "Git.Git" }
        if (-not (Has "git")) { $env:Path += ";C:\Program Files\Git\cmd" }
        $Drive = if (Test-Path "D:\") { "D:" } else { "C:" }
        $Proj = "$Drive\NIDAR_DRONE_PROJECT"
        if (Test-Path (Join-Path $Proj ".git")) {
            Say "    updating $Proj"
            git -C $Proj pull --ff-only
        } else {
            Say "    downloading the project into $Proj"
            # LF line endings for the Linux scripts are pinned by the project's .gitattributes
            git clone $RepoUrl $Proj
            if ($LASTEXITCODE -ne 0) { Warn "git clone failed (internet? private repository?)"; return }
        }
    }

    # ---------------------------------------------------------- 2. Python + YOLO
    Say "2/3 Python 3.12 + camera AI packages (about 1-2 GB the first time)"
    if (-not (Has "py")) { Say "    installing Python 3.12"; Winget-Install "Python.Python.3.12" }
    if (-not (Has "py")) { Warn "Python launcher 'py' not found: close PowerShell, open it again as administrator, run this again."; return }
    py -3 -m pip install --upgrade pip
    py -3 -m pip install -r (Join-Path $Proj "requirements-windows.txt")
    if ($LASTEXITCODE -ne 0) { Warn "pip install failed (see above)"; return }
    py -3 -c "import ultralytics, cv2, numpy; print('    ultralytics', ultralytics.__version__, '| OpenCV', cv2.__version__, '| numpy', numpy.__version__)"
    if (-not (Test-Path (Join-Path $Proj "yoloe-26n-seg.pt"))) { Warn "yoloe-26n-seg.pt is missing in $Proj" }

    # ---------------------------------------------------------- 3. WSL + Ubuntu 24.04
    Say "3/3 WSL 2 + Ubuntu 24.04"
    $distros = ((wsl.exe -l -q 2>$null) -join "`n") -replace "`0", ""
    $restart = $false
    if ($distros -match "Ubuntu-24\.04") {
        Say "    Ubuntu-24.04 is already installed"
    } else {
        wsl.exe --install -d Ubuntu-24.04 --no-launch
        if ($LASTEXITCODE -ne 0) { wsl.exe --install -d Ubuntu-24.04 }
        $restart = $true
    }
    wsl.exe --update 2>$null | Out-Null                     # newest WSL: windows of Linux apps (Gazebo, the app) work
    wsl.exe --set-default Ubuntu-24.04 2>$null | Out-Null

    $WslProj = "/mnt/" + $Proj.Substring(0, 1).ToLower() + ($Proj.Substring(2) -replace "\\", "/")
    Write-Host ""
    Say "Windows part DONE. Next:"
    if ($restart) { Say "  a) RESTART Windows now (WSL needs it the first time)." }
    Say "  b) Start menu -> 'Ubuntu 24.04' -> choose a user name + password (remember the password)."
    Say "  c) In that Ubuntu window type (about 1 hour, keep the laptop plugged in):"
    Write-Host ""
    Write-Host "     bash $WslProj/tools/install_wsl.sh" -ForegroundColor Green
    Write-Host ""
}

Install-Nidar
