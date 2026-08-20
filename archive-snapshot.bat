@echo off
REM Archive script for OptAM-MPC
REM Usage: archive-snapshot.bat "description"
REM Creates a snapshot in the archive folder inside the repo

setlocal enabledelayedexpansion

REM Get timestamp using PowerShell
for /f "delims=" %%I in ('powershell -Command "Get-Date -Format 'yyyyMMdd-HHmmss'"') do set timestamp=%%I

REM Get description from command line or use timestamp only
if "%~1"=="" (
    set foldername=!timestamp!
) else (
    set foldername=!timestamp!-%~1
)

REM Create archive folder inside repo
set archivepath=C:\Users\Kobus\optam-mpc\archive\!foldername!
mkdir "!archivepath!"

REM Copy files (excluding unnecessary folders)
robocopy C:\Users\Kobus\optam-mpc "!archivepath!" /E /XD venv __pycache__ .git logs .pytest_cache archive

echo.
echo Archive created: !archivepath!
echo.

endlocal