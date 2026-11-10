# pull_poses.ps1 - copy the arm data from the Raspberry Pi to this laptop as a backup.
#
# Run from PowerShell:
#     powershell -ExecutionPolicy Bypass -File C:\Users\arago\engg\RoboComp\pull_poses.ps1
#
# Every run makes NEW files in RoboComp\backups\ with the date and time in the
# name, so old backups are never overwritten. It only reads from the Pi.

# ---- edit these 3 lines ----
$PiUser   = "engg1101n2"
$PiHost   = "ENGG1101-N2"               # if the name doesn't work, put the Pi's IP here, e.g. "192.168.1.42"
$RepoPath = "STS3215-5DOF-Robot-Arm"    # folder on the Pi, relative to the home folder
# ----------------------------

$Files = @("poses.json", "game_state.json")
$BackupDir = Join-Path $PSScriptRoot "backups"
$Stamp = Get-Date -Format "yyyy-MM-dd_HHmm"

New-Item -ItemType Directory -Force -Path $BackupDir | Out-Null

Write-Host "Copying from $PiUser@$PiHost ..."
$copied = 0
foreach ($f in $Files) {
    $base = [System.IO.Path]::GetFileNameWithoutExtension($f)
    $dest = Join-Path $BackupDir "${base}_$Stamp.json"
    $src  = "${PiUser}@${PiHost}:$RepoPath/game/$f"
    & scp -q -o ConnectTimeout=8 $src $dest
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  saved $dest"
        $copied++
    } else {
        Write-Host "  could not copy $f (see the message above)"
    }
}

if ($copied -eq 0) {
    Write-Host ""
    Write-Host "Nothing copied. Check:"
    Write-Host "  1. Pi is on and on the same Wi-Fi:  ping $PiHost"
    Write-Host "  2. If the name fails, put the Pi's IP address in `$PiHost at the top of this file"
    Write-Host "  3. The repo folder is right. Find it with:"
    Write-Host "       ssh $PiUser@$PiHost `"find ~ -name arm_helpers.py`""
    Write-Host "  4. poses.json only exists after you 'save' a pose with DRY_RUN = False"
    exit 1
}
Write-Host "Done: $copied file(s) backed up."
