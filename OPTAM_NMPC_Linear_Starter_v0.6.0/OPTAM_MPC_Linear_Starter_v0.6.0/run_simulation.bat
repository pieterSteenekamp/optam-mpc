@echo off
setlocal
if "%~2"=="" goto :usage
if not exist "%~dp0.venv\Scripts\python.exe" goto :setup
if "%~3"=="" (
  "%~dp0.venv\Scripts\python.exe" "%~dp0run_simulation.py" "%~f1" "%~f2"
) else (
  "%~dp0.venv\Scripts\python.exe" "%~dp0run_simulation.py" "%~f1" "%~f2" --output "%~f3"
)
exit /b %errorlevel%
:usage
echo Usage: run_simulation.bat application.toml simulation.toml [results_folder]
exit /b 1
:setup
echo Run setup_windows.bat first.
exit /b 1
