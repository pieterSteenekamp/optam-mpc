@echo off
setlocal
if "%~3"=="" goto :usage
if not exist "%~dp0.venv\Scripts\python.exe" goto :setup
"%~dp0.venv\Scripts\python.exe" "%~dp0run_opc_twin.py" "%~f1" "%~f2" "%~f3"
exit /b %errorlevel%
:usage
echo Usage: run_digital_twin.bat application.toml simulation.toml opc.toml
exit /b 1
:setup
echo Run setup_windows.bat first.
exit /b 1
