@echo off
REM Build/upload (optional) + serial monitor for the ESP32 CSI test-node.
REM
REM Usage:
REM   run.bat            -- just attach the serial monitor (default)
REM   run.bat --build    -- build, upload, then attach the serial monitor
REM
REM Edit ENV/PORT below to match your board (see README.md "Identifying
REM which board you have").

setlocal

set ENV=esp32-c3-devkitm-1
set PORT=COM9

cd /d %~dp0

set PIO=%USERPROFILE%\.platformio\penv\Scripts\pio.exe
if not exist "%PIO%" (
    where pio >nul 2>nul
    if errorlevel 1 (
        echo [ERROR] Could not find pio.exe on PATH or at "%PIO%".
        echo         Install PlatformIO or add its Scripts dir to PATH.
        exit /b 1
    )
    set PIO=pio
)

set DO_BUILD=0
:parse_args
if "%~1"=="" goto args_done
if /I "%~1"=="--build" set DO_BUILD=1
shift
goto parse_args
:args_done

if %DO_BUILD%==1 (
    echo === Building and uploading [env=%ENV% port=%PORT%] ===
    "%PIO%" run -e %ENV% -t upload --upload-port %PORT%
    if errorlevel 1 (
        echo [ERROR] Build/upload failed.
        exit /b 1
    )
)

echo === Serial monitor [port=%PORT%] — Ctrl+C to quit ===
"%PIO%" device monitor -b 115200 -p %PORT%

endlocal
