@echo off
rem start_debug.bat - double-click to start Mosquitto, car_link.py and the
rem dashboard for PicoCar Wi-Fi debugging (see start_debug.ps1).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_debug.ps1"
if errorlevel 1 pause
