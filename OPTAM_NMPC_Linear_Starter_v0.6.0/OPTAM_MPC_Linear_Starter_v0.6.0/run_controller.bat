@echo off
setlocal
if "%~2"=="" goto :usage
if not exist "%~dp0.venv\Scripts\python.exe" goto :setup
"%~dp0.venv\Scripts\python.exe" "%~dp0run_opc_controller.py" "%~f1" "%~f2"
exit /b %errorlevel%
:usage
echo Usage: run_controller.bat application.toml opc.toml
exit /b 1
:setup
echo Run setup_windows.bat first.
exit /b 1
