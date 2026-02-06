#!/bin/bash
# USB deploy helper for FTC TeamCode on REV Control Hub (macOS/Linux)
# Deploys via USB cable connection instead of Wi-Fi
#
# Usage: scripts/usb_deploy.sh

set -e

# Resolve repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

echo "[deploy] Repository: $(pwd)"
echo "[deploy] Connection: USB"
echo ""

# Find ADB
ADB=""
if [ -n "$ANDROID_HOME" ] && [ -x "$ANDROID_HOME/platform-tools/adb" ]; then
    ADB="$ANDROID_HOME/platform-tools/adb"
elif [ -x "$HOME/Library/Android/sdk/platform-tools/adb" ]; then
    ADB="$HOME/Library/Android/sdk/platform-tools/adb"
elif [ -x "/usr/local/bin/adb" ]; then
    ADB="/usr/local/bin/adb"
elif [ -x "/opt/homebrew/bin/adb" ]; then
    ADB="/opt/homebrew/bin/adb"
elif command -v adb &> /dev/null; then
    ADB="$(command -v adb)"
fi

if [ -z "$ADB" ]; then
    echo "[ERROR] adb not found."
    echo ""
    echo "Please install Android SDK Platform Tools:"
    echo "  brew install android-platform-tools"
    echo ""
    echo "Or download from:"
    echo "  https://developer.android.com/studio/releases/platform-tools"
    echo ""
    echo "Or set ANDROID_HOME environment variable to your Android SDK location"
    exit 1
fi

echo "[deploy] Using ADB: $ADB"
echo ""

# Check for USB connected devices
echo "[deploy] Checking for USB connected devices..."
"$ADB" kill-server > /dev/null 2>&1 || true
"$ADB" start-server > /dev/null 2>&1 || true
sleep 1

# Get list of USB devices (non-network devices)
DEVICES=$("$ADB" devices | grep -v ":" | grep "device$" | awk '{print $1}')

if [ -z "$DEVICES" ]; then
    echo "[ERROR] No USB devices found"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Connect REV Control Hub via USB cable"
    echo "  2. Ensure the USB cable supports data transfer (not charge-only)"
    echo "  3. Check that the Control Hub is powered on"
    echo "  4. Try a different USB port"
    echo "  5. On macOS, check System Information > USB for the device"
    echo ""
    echo "Current ADB devices:"
    "$ADB" devices
    exit 1
fi

# Count devices
DEVICE_COUNT=$(echo "$DEVICES" | wc -l | tr -d ' ')

if [ "$DEVICE_COUNT" -gt 1 ]; then
    echo "[WARNING] Multiple USB devices found:"
    echo "$DEVICES"
    echo ""
    echo "Using first device. To specify a device, set ANDROID_SERIAL:"
    echo "  export ANDROID_SERIAL=<device_id>"
    DEVICE=$(echo "$DEVICES" | head -n 1)
else
    DEVICE="$DEVICES"
fi

echo "[deploy] Device: $DEVICE"

# Get device info
MODEL=$("$ADB" -s "$DEVICE" shell getprop ro.product.model 2>/dev/null | tr -d '\r' || echo "Unknown")
echo "[deploy] Model: $MODEL"
echo ""

# Wait for device to be fully ready
echo "[deploy] Waiting for device to be ready..."
WAIT_COUNT=0
while [ $WAIT_COUNT -lt 5 ]; do
    STATE=$("$ADB" -s "$DEVICE" get-state 2>/dev/null | tr -d '\r' || echo "")
    if [ "$STATE" = "device" ]; then
        echo "[deploy] Device ready"
        break
    fi
    
    WAIT_COUNT=$((WAIT_COUNT + 1))
    echo "[deploy] Waiting... ($WAIT_COUNT/5)"
    sleep 2
done

if [ $WAIT_COUNT -ge 5 ]; then
    echo "[ERROR] Device not responding"
    "$ADB" devices
    exit 1
fi

# Build and install
echo ""
echo "[deploy] Building and installing..."
export ANDROID_SERIAL="$DEVICE"

if ! ./gradlew :TeamCode:installDebug; then
    echo ""
    echo "[ERROR] Build or install failed"
    exit 1
fi

echo ""
echo "[deploy] ✓ Deploy successful via USB"
echo "[deploy] Device: $DEVICE ($MODEL)"
exit 0
