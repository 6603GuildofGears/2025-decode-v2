#!/bin/bash
# Wi-Fi deploy helper for FTC TeamCode on REV Control Hub (macOS/Linux)
# Based on Android Studio's deployment approach
#
# Usage: scripts/wifi_deploy.sh [ip:port]
# Default target: 192.168.43.1:5555

set -e

DEFAULT_TARGET="192.168.43.1:5555"
TARGET="${1:-$DEFAULT_TARGET}"

# Resolve repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

echo "[deploy] Repository: $(pwd)"
echo "[deploy] Target: $TARGET"
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

# Ensure device is connected and online
echo "[deploy] Establishing connection..."

# First, disconnect any stale connections
"$ADB" disconnect "$TARGET" > /dev/null 2>&1 || true

# Connect fresh
"$ADB" connect "$TARGET" > /dev/null 2>&1 || true
sleep 2

# Wait for device to be ready (up to 10 seconds)
WAIT_COUNT=0
while [ $WAIT_COUNT -lt 5 ]; do
    if "$ADB" -s "$TARGET" get-state > /dev/null 2>&1; then
        echo "[deploy] Device ready"
        break
    fi
    
    WAIT_COUNT=$((WAIT_COUNT + 1))
    echo "[deploy] Waiting for device... ($WAIT_COUNT/5)"
    sleep 2
done

if [ $WAIT_COUNT -ge 5 ]; then
    echo "[ERROR] Device not responding"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Verify Control Hub WiFi is active"
    echo "  2. Check computer is on Control Hub network"
    echo "  3. Try: ping ${TARGET%:*}"
    echo "  4. Power cycle the Control Hub"
    "$ADB" devices
    exit 1
fi

# Build and install
echo ""
echo "[deploy] Building and installing..."
if ! ./gradlew :TeamCode:installDebug; then
    echo ""
    echo "[ERROR] Build or install failed"
    exit 1
fi

echo ""
echo "[deploy] ✓ Deploy successful"
echo "[deploy] Connection maintained for debugging"
exit 0
