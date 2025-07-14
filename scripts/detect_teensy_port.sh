#!/bin/bash

# Script to detect Teensy port automatically

detect_teensy_port() {
    echo "Detecting Teensy port..."
    
    # Check for Teensy by looking for specific vendor/product IDs
    for port in /dev/ttyACM*; do
        if [ -e "$port" ]; then
            # Get USB device info
            port_num=$(basename "$port" | sed 's/ttyACM//')
            usb_device_path="/sys/class/tty/ttyACM${port_num}/device/uevent"
            
            if [ -f "$usb_device_path" ]; then
                # Check if it's a Teensy device (vendor ID 16c0)
                if grep -q "PRODUCT=16c0" "$usb_device_path" 2>/dev/null; then
                    echo "Found Teensy on $port"
                    echo "$port"
                    return 0
                fi
            fi
        fi
    done
    
    # Fallback: check for any ACM device and test if it has Teensy-like behavior
    for port in /dev/ttyACM*; do
        if [ -e "$port" ]; then
            echo "Found ACM device on $port (assuming Teensy)"
            echo "$port"
            return 0
        fi
    done
    
    echo "No Teensy device found!" >&2
    return 1
}

# Main execution
TEENSY_PORT=$(detect_teensy_port)
if [ $? -eq 0 ]; then
    echo "TEENSY_PORT=$TEENSY_PORT"
    export TEENSY_PORT
else
    echo "Failed to detect Teensy port"
    exit 1
fi
