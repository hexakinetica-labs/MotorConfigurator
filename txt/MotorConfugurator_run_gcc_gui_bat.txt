@echo off
setlocal enableextensions

REM Run the already built Qt GUI with the pinned Qt MinGW runtime paths.

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

set "QT_ROOT=C:\Qt"
set "MINGW_ROOT=%QT_ROOT%\Tools\mingw1310_64"
set "QT_PREFIX=%QT_ROOT%\6.11.1\mingw_64"
set "BUILD_DIR=%SCRIPT_DIR%\build-gcc"
set "APP_EXE=%BUILD_DIR%\motor_tester_gui.exe"
set "LIBUSB_DLL=%BUILD_DIR%\libusb-1.0.dll"

echo [INFO] Working directory: %SCRIPT_DIR%

if not exist "%APP_EXE%" (
    echo [ERROR] Application not found: %APP_EXE%
    echo [HINT] Run build_gcc_gui.bat first.
    exit /b 1
)

if not exist "%QT_PREFIX%\bin\Qt6Core.dll" (
    echo [ERROR] Qt MinGW runtime not found under: %QT_PREFIX%
    exit /b 2
)

if not exist "%LIBUSB_DLL%" (
    echo [ERROR] Missing deployed libusb runtime DLL: %LIBUSB_DLL%
    echo [HINT] Re-run build_gcc_gui.bat to redeploy runtime files.
    exit /b 3
)

set "PATH=%BUILD_DIR%;%QT_PREFIX%\bin;%MINGW_ROOT%\bin;%PATH%"
echo [INFO] Launching motor_tester_gui.exe...
start "motor_tester_gui" "%APP_EXE%"
exit /b 0