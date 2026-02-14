#!/bin/bash
# USB deploy helper for FTC TeamCode on REV Control Hub (macOS)
# Deploys over a direct USB connection (no Wi-Fi needed)
#
# Usage: scripts/usb_deploy.sh

set -e

# Resolve repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

echo "[usb-deploy] Repository: $(pwd)"
echo ""

# ── Find ADB ──
ADB=""
if [ -n "$ANDROID_HOME" ] && [ -x "$ANDROID_HOME/platform-tools/adb" ]; then
    ADB="$ANDROID_HOME/platform-tools/adb"
elif [ -x "$HOME/Library/Android/sdk/platform-tools/adb" ]; then
    ADB="$HOME/Library/Android/sdk/platform-tools/adb"
elif [ -x "/opt/homebrew/bin/adb" ]; then
    ADB="/opt/homebrew/bin/adb"
elif [ -x "/usr/local/bin/adb" ]; then
    ADB="/usr/local/bin/adb"
elif command -v adb &> /dev/null; then
    ADB="$(command -v adb)"
fi

if [ -z "$ADB" ]; then
    echo "[ERROR] adb not found."
    echo ""
    echo "Install Android SDK Platform Tools:"
    echo "  brew install android-platform-tools"
    echo ""
    echo "Or download from:"
    echo "  https://developer.android.com/studio/releases/platform-tools"
    echo ""
    echo "Or set ANDROID_HOME to your Android SDK location."
    exit 1
fi

echo "[usb-deploy] Using ADB: $ADB"
echo ""

# ── Look for a USB device (restart ADB only if needed) ──
echo "[usb-deploy] Checking for USB device..."
WAIT_COUNT=0
DEVICE_SERIAL=""

# First, try without restarting ADB
DEVICE_SERIAL=$("$ADB" devices | grep -v '^\s*$' | grep -v '^List' | grep -v ':' | awk '{print $1}' | head -n 1)

if [ -z "$DEVICE_SERIAL" ]; then
    echo "[usb-deploy] No device found, restarting ADB server..."
    "$ADB" kill-server > /dev/null 2>&1 || true
    "$ADB" start-server > /dev/null 2>&1
    sleep 2
fi

while [ $WAIT_COUNT -lt 10 ]; do
    # List only USB-connected devices (ignore network ones with ":" in serial)
    DEVICE_SERIAL=$("$ADB" devices | grep -v '^\s*$' | grep -v '^List' | grep -v ':' | awk '{print $1}' | head -n 1)

    if [ -n "$DEVICE_SERIAL" ]; then
        echo "[usb-deploy] Found device: $DEVICE_SERIAL"
        break
    fi

    WAIT_COUNT=$((WAIT_COUNT + 1))
    echo "[usb-deploy] Waiting... ($WAIT_COUNT/10)  — Is the Control Hub plugged in via USB?"
    sleep 2
done

if [ -z "$DEVICE_SERIAL" ]; then
    echo ""
    echo "[ERROR] No USB device found after 20 seconds."
    echo ""
    echo "Troubleshooting:"
    echo "  1. Connect the Control Hub to your Mac with a USB-C cable"
    echo "  2. Make sure the Control Hub is powered on"
    echo "  3. Try a different USB port or cable"
    echo "  4. Run: adb devices   (to check manually)"
    echo ""
    "$ADB" devices
    exit 1
fi

# ── Verify device is online ──
DEVICE_STATE=$("$ADB" -s "$DEVICE_SERIAL" get-state 2>/dev/null || echo "offline")
if [ "$DEVICE_STATE" != "device" ]; then
    echo "[ERROR] Device is connected but state is: $DEVICE_STATE"
    echo "Try unplugging and re-plugging the USB cable."
    exit 1
fi

echo "[usb-deploy] Device online ✓"
echo ""

# ── Build and install over USB ──
echo "[usb-deploy] Building and installing via USB..."
if ! ANDROID_SERIAL="$DEVICE_SERIAL" ./gradlew :TeamCode:installDebug; then
    echo ""
    echo "[ERROR] Build or install failed."
    exit 1
fi

echo ""
echo "[usb-deploy] ✅ Deploy successful over USB!"
echo "[usb-deploy] Device: $DEVICE_SERIAL"
exit 0
