@echo off
setlocal enableextensions

REM Build and deploy the Qt GUI with the pinned Qt MinGW toolchain.
REM This script is intentionally explicit so operator machines do not depend on PATH state.

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

set "QT_ROOT=C:\Qt"
set "CMAKE_EXE=%QT_ROOT%\Tools\CMake_64\bin\cmake.exe"
set "MINGW_ROOT=%QT_ROOT%\Tools\mingw1310_64"
set "GCC_EXE=%MINGW_ROOT%\bin\gcc.exe"
set "GXX_EXE=%MINGW_ROOT%\bin\g++.exe"
set "MINGW_MAKE_EXE=%MINGW_ROOT%\bin\mingw32-make.exe"
set "QT_PREFIX=%QT_ROOT%\6.11.1\mingw_64"
set "WINDEPLOYQT_EXE=%QT_PREFIX%\bin\windeployqt.exe"

set "BUILD_DIR=%SCRIPT_DIR%\build-gcc"
set "APP_EXE=%BUILD_DIR%\motor_tester_gui.exe"

set "PATH=%MINGW_ROOT%\bin;%QT_PREFIX%\bin;%QT_ROOT%\Tools\CMake_64\bin;%PATH%"

set "LIBUSB_HEADER=%SCRIPT_DIR%\3rdparty\win\libusb\include\libusb-1.0\libusb.h"
set "LIBUSB_IMPORT_LIB=%SCRIPT_DIR%\3rdparty\win\libusb\lib\x64\libusb-1.0.dll.a"
set "LIBUSB_DLL=%SCRIPT_DIR%\3rdparty\win\libusb\bin\x64\libusb-1.0.dll"

echo [INFO] Working directory: %SCRIPT_DIR%
echo [INFO] MinGW bin in PATH: %MINGW_ROOT%\bin
echo [INFO] Qt bin in PATH: %QT_PREFIX%\bin

if not exist "%CMAKE_EXE%" (
    echo [ERROR] CMake not found: %CMAKE_EXE%
    exit /b 1
)

if not exist "%GCC_EXE%" (
    echo [ERROR] GCC not found: %GCC_EXE%
    exit /b 1
)

if not exist "%GXX_EXE%" (
    echo [ERROR] G++ not found: %GXX_EXE%
    exit /b 1
)

if not exist "%MINGW_MAKE_EXE%" (
    echo [ERROR] MinGW Make not found: %MINGW_MAKE_EXE%
    exit /b 1
)

if not exist "%QT_PREFIX%\bin\Qt6Core.dll" (
    echo [ERROR] Qt MinGW runtime not found under: %QT_PREFIX%
    exit /b 1
)

if not exist "%LIBUSB_HEADER%" (
    echo [ERROR] Missing Windows libusb header: %LIBUSB_HEADER%
    echo [HINT] The current Windows MinGW build requires vendored libusb files under 3rdparty\win\libusb.
    exit /b 2
)

if not exist "%LIBUSB_IMPORT_LIB%" (
    echo [ERROR] Missing Windows libusb import library: %LIBUSB_IMPORT_LIB%
    echo [HINT] The current Windows MinGW build requires vendored libusb files under 3rdparty\win\libusb.
    exit /b 2
)

if not exist "%LIBUSB_DLL%" (
    echo [ERROR] Missing Windows libusb runtime DLL: %LIBUSB_DLL%
    echo [HINT] The current Windows MinGW build requires vendored libusb files under 3rdparty\win\libusb.
    exit /b 2
)

echo [INFO] Configuring with Qt MinGW toolchain...
"%CMAKE_EXE%" -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -G "MinGW Makefiles" ^
    -DCMAKE_C_COMPILER="%GCC_EXE%" ^
    -DCMAKE_CXX_COMPILER="%GXX_EXE%" ^
    -DCMAKE_MAKE_PROGRAM="%MINGW_MAKE_EXE%" ^
    -DCMAKE_PREFIX_PATH="%QT_PREFIX%" ^
    -DMOTORCONF_ENABLE_ETHERCAT=OFF ^
    -DMOTORCONF_ENABLE_MKS_CAN=ON
if errorlevel 1 (
    echo [ERROR] CMake configure failed.
    exit /b 3
)

echo [INFO] Building motor_tester_gui...
"%CMAKE_EXE%" --build "%BUILD_DIR%" --target motor_tester_gui -- -j2
if errorlevel 1 (
    echo [ERROR] Build failed.
    exit /b 4
)

if not exist "%APP_EXE%" (
    echo [ERROR] Build completed without expected executable: %APP_EXE%
    exit /b 5
)

if exist "%WINDEPLOYQT_EXE%" (
    echo [INFO] Deploying Qt runtime files...
    "%WINDEPLOYQT_EXE%" --no-compiler-runtime "%APP_EXE%"
    if errorlevel 1 (
        echo [WARN] windeployqt reported an error. Continuing anyway.
    )
) else (
    echo [WARN] windeployqt not found: %WINDEPLOYQT_EXE%
)

echo [INFO] Copying libusb runtime DLL next to the executable...
copy /Y "%LIBUSB_DLL%" "%BUILD_DIR%\libusb-1.0.dll" >nul
if errorlevel 1 (
    echo [ERROR] Failed to copy libusb runtime DLL.
    exit /b 6
)

echo [INFO] Build and deploy completed successfully.
echo [INFO] Use run_gcc_gui.bat to start the application.
exit /b 0