#!/bin/bash
set -e

echo "=========================================="
echo "ST-Link V2 Debugger - USB Access Setup"
echo "=========================================="
echo ""

# ST-Link V2 and V2-1 rules
UDEV_RULES='# ST-Link V2
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0666", TAG+="uaccess"
# ST-Link V2-1
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374b", MODE="0666", TAG+="uaccess"
# ST-Link V3
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374e", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374f", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3753", MODE="0666", TAG+="uaccess"'

RULES_FILE="/etc/udev/rules.d/99-stlink.rules"

# Check if we're running inside a container
if [ -f /.dockerenv ] || grep -q docker /proc/1/cgroup 2>/dev/null; then
    echo "⚠️  Running inside container - cannot directly modify host udev rules"
    echo ""
    echo "To enable USB access for ST-Link on your HOST machine:"
    echo ""
    echo "1. Run these commands on your HOST (outside the container):"
    echo ""
    echo "   sudo tee /etc/udev/rules.d/99-stlink.rules > /dev/null <<'EOF'"
    echo "$UDEV_RULES"
    echo "EOF"
    echo ""
    echo "   sudo udevadm control --reload-rules"
    echo "   sudo udevadm trigger"
    echo ""
    echo "2. Reconnect your ST-Link V2"
    echo ""
    echo "3. Verify with: lsusb | grep 0483"
    echo ""
    
    # Check if ST-Link device is visible
    if lsusb 2>/dev/null | grep -q "0483:3748"; then
        echo "✓ ST-Link V2 device detected (0483:3748)"
        
        if [ -w /dev/bus/usb/*/*/* ] 2>/dev/null; then
            echo "✓ USB device access appears to be working"
        else
            echo "⚠️  USB device detected but may not be accessible"
            echo "   Please run the udev setup commands above on your host"
        fi
    else
        echo "⚠️  ST-Link V2 device not detected"
        echo "   Make sure your ST-Link is connected"
    fi
else
    # Running on host
    echo "Running on host machine - setting up udev rules..."
    
    if [ "$EUID" -ne 0 ]; then
        echo "⚠️  This script needs root privileges to modify udev rules"
        echo "Please run: sudo $0"
        exit 1
    fi
    
    echo "$UDEV_RULES" > "$RULES_FILE"
    echo "✓ Created $RULES_FILE"
    
    udevadm control --reload-rules
    udevadm trigger
    echo "✓ Reloaded udev rules"
    echo ""
    echo "✓ Setup complete! Reconnect your device if it's already plugged in."
fi

echo ""
echo "=========================================="