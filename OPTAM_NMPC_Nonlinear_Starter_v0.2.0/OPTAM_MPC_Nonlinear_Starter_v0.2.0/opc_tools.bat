@echo off
setlocal
if "%~2"=="" goto :usage
if not exist "%~dp0.venv\Scripts\python.exe" goto :setup
if "%~3"=="" (
  "%~dp0.venv\Scripts\python.exe" "%~dp0opc_tools.py" "%~f1" "%~2"
) else (
  "%~dp0.venv\Scripts\python.exe" "%~dp0opc_tools.py" "%~f1" "%~2" "%~f3"
)
exit /b %errorlevel%
:usage
echo Usage: opc_tools.bat opc.toml status / stop / trend [simulation.toml]
exit /b 1
:setup
echo Run setup_windows.bat first.
exit /b 1
