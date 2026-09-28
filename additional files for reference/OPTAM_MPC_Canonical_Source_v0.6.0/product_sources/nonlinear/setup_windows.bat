@echo off
setlocal
python -c "import sys; assert sys.version_info >= (3,11), 'Python 3.11 or later required'"
if errorlevel 1 exit /b 1
python -m venv "%~dp0.venv"
if errorlevel 1 exit /b 1
"%~dp0.venv\Scripts\python.exe" -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 exit /b 1
echo Setup complete. See USER_MANUAL.md for generic operating commands.
