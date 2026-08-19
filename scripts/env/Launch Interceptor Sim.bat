@echo off
title Interceptor Sim Launcher
set "VBOX=C:\Program Files\Oracle\VirtualBox\VBoxManage.exe"
if not exist "%VBOX%" (
  echo ERROR: VirtualBox was not found at "%VBOX%".
  echo Please reinstall VirtualBox, then run this launcher again.
  pause
  exit /b 1
)
echo ==================================================
echo    Interceptor Sim  -  launching environment
echo ==================================================
echo.
echo Starting the virtual machine (instant if already running)...
"%VBOX%" startvm interceptor-sim --type headless 2>nul
echo Waiting for the simulation terminal to come online...
set /a tries=0
:wait
powershell -NoProfile -Command "try{$c=New-Object Net.Sockets.TcpClient;$c.Connect('127.0.0.1',7681);$c.Close();exit 0}catch{exit 1}" >nul 2>&1
if not errorlevel 1 goto ready
set /a tries+=1
if %tries% geq 90 (
  echo.
  echo The VM is still booting. Please wait a minute and re-run this launcher.
  pause
  exit /b 1
)
timeout /t 2 >nul
goto wait
:ready
echo Ready! Opening Claude Code in your browser...
start "" "http://localhost:7681"
timeout /t 3 >nul
exit
