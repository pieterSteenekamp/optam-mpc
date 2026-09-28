@echo off
setlocal
if "%~1"=="" goto :usage
if not exist "%~dp0.venv\Scripts\python.exe" goto :setup
if not "%~3"=="" (
  "%~dp0.venv\Scripts\python.exe" "%~dp0validate_and_report.py" "%~f1" "%~f2" "%~f3"
) else if "%~2"=="" (
  "%~dp0.venv\Scripts\python.exe" "%~dp0validate_and_report.py" "%~f1"
) else (
  "%~dp0.venv\Scripts\python.exe" "%~dp0validate_and_report.py" "%~f1" "%~f2"
)
exit /b %errorlevel%
:usage
echo Usage: validate_application.bat application.toml [simulation.toml] [opc.toml]
exit /b 1
:setup
echo Run setup_windows.bat first.
exit /b 1
