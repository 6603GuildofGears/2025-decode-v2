@echo off
REM Wi-Fi deploy helper for FTC TeamCode on REV Control Hub (Windows)
REM Based on Android Studio's deployment approach
REM
REM Usage: scripts\wifi_deploy.bat [ip:port]
REM Default target: 192.168.43.1:5555

setlocal EnableDelayedExpansion

set DEFAULT_TARGET=192.168.43.1:5555
set TARGET=%DEFAULT_TARGET%

REM Parse target if provided
if not "%~1"=="" (
    set TARGET=%~1
)

REM Resolve repo root
set SCRIPT_DIR=%~dp0
set REPO_ROOT=%SCRIPT_DIR%..
cd /d "%REPO_ROOT%"

echo [deploy] Repository: %CD%
echo [deploy] Target: %TARGET%
echo.

REM Connect to device
set ADB=
if exist "%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe" (
    set ADB=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe
) else if exist "%USERPROFILE%\AppData\Local\Android\Sdk\platform-tools\adb.exe" (
    set ADB=%USERPROFILE%\AppData\Local\Android\Sdk\platform-tools\adb.exe
) else if exist "%ANDROID_HOME%\platform-tools\adb.exe" (
    set ADB=%ANDROID_HOME%\platform-tools\adb.exe
) else if exist "C:\Users\%USERNAME%\AppData\Local\Android\Sdk\platform-tools\adb.exe" (
    set ADB=C:\Users\%USERNAME%\AppData\Local\Android\Sdk\platform-tools\adb.exe
) else (
    where adb.exe >nul 2>&1
    if !errorlevel!==0 (
        for /f "delims=" %%i in ('where adb.exe') do set ADB=%%i
    )
)

if "%ADB%"=="" (
    echo [ERROR] adb.exe not found. 
    echo.
    echo Please install Android SDK Platform Tools from:
    echo https://developer.android.com/studio/releases/platform-tools
    echo.
    echo Or set ANDROID_HOME environment variable to your Android SDK location
    pause
    exit /b 1
)

echo [deploy] Using ADB: %ADB%
echo.

REM Check if device is already connected
"%ADB%" -s %TARGET% shell echo ok >nul 2>&1
if !errorlevel! equ 0 (
    echo [deploy] Device already connected
    goto device_ready
)

REM Try quick reconnect first
echo [deploy] Device not responding, attempting reconnect...
"%ADB%" reconnect device >nul 2>&1
timeout /t 3 /nobreak >nul
"%ADB%" -s %TARGET% shell echo ok >nul 2>&1
if !errorlevel! equ 0 (
    echo [deploy] Reconnected successfully
    goto device_ready
)

REM Full connection sequence
echo [deploy] Reconnect failed, establishing fresh connection...

REM Disconnect any stale connections
"%ADB%" disconnect %TARGET% >nul 2>&1

REM Connect fresh
"%ADB%" connect %TARGET% >nul 2>&1
timeout /t 2 /nobreak >nul

REM Wait for device to be ready (up to 10 seconds)
set WAIT_COUNT=0
:wait_for_device
"%ADB%" -s %TARGET% wait-for-device >nul 2>&1
if !errorlevel! equ 0 (
    echo [deploy] Device ready
    goto device_ready
)

set /a WAIT_COUNT+=1
if !WAIT_COUNT! lss 5 (
    echo [deploy] Waiting for device... (!WAIT_COUNT!/5^)
    timeout /t 2 /nobreak >nul
    goto wait_for_device
)

echo [ERROR] Device not responding
echo.
echo Troubleshooting:
echo   1. Verify Control Hub WiFi is active
echo   2. Check computer is on Control Hub network  
echo   3. Try: ping %TARGET:~0,-5%
echo   4. Power cycle the Control Hub
"%ADB%" devices
exit /b 1

:device_ready

REM Build and install
echo.
echo [deploy] Building and installing...
call gradlew.bat :TeamCode:installDebug
if !errorlevel! neq 0 (
    echo.
    echo [WARN] Install failed, checking for signature mismatch...
    
    REM Try uninstalling and reinstalling if signature mismatch
    "%ADB%" -s %TARGET% uninstall com.qualcomm.ftcrobotcontroller >nul 2>&1
    if !errorlevel! equ 0 (
        echo [deploy] Removed old version, retrying install...
        call gradlew.bat :TeamCode:installDebug
        if !errorlevel! neq 0 (
            echo [ERROR] Install failed after uninstall
            exit /b 1
        )
    ) else (
        echo [ERROR] Build or install failed
        exit /b 1
    )
)

echo.
echo [deploy] ✓ Deploy successful
echo [deploy] Restarting Robot Controller app...

REM Force stop and restart the app
"%ADB%" -s %TARGET% shell am force-stop com.qualcomm.ftcrobotcontroller
timeout /t 2 /nobreak >nul
"%ADB%" -s %TARGET% shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity

echo [deploy] ✓ App restarted with new code
echo [deploy] Connection maintained for debugging
exit /b 0
