#!/bin/bash

# JiggyJoystick System Startup Script
# Automatically detects Teensy port and starts Docker services

set -e

echo "🚀 Starting JiggyJoystick System..."

# Detect Teensy port
echo "📍 Detecting Teensy port..."
TEENSY_PORT=""

# Check for Teensy by looking for specific vendor/product IDs
for port in /dev/ttyACM*; do
    if [ -e "$port" ]; then
        # Get USB device info
        port_num=$(basename "$port" | sed 's/ttyACM//')
        usb_device_path="/sys/class/tty/ttyACM${port_num}/device/uevent"
        
        if [ -f "$usb_device_path" ]; then
            # Check if it's a Teensy device (vendor ID 16c0)
            if grep -q "PRODUCT=16c0" "$usb_device_path" 2>/dev/null; then
                echo "✅ Found Teensy on $port"
                TEENSY_PORT="$port"
                break
            fi
        fi
    fi
done

# Fallback: check for any ACM device
if [ -z "$TEENSY_PORT" ]; then
    for port in /dev/ttyACM*; do
        if [ -e "$port" ]; then
            echo "⚠️  Found ACM device on $port (assuming Teensy)"
            TEENSY_PORT="$port"
            break
        fi
    done
fi

# Error if no port found
if [ -z "$TEENSY_PORT" ]; then
    echo "❌ No Teensy device found!"
    echo "Please check that the Teensy is connected and try again."
    exit 1
fi

# Export the port for docker-compose
export TEENSY_PORT

echo "🔧 Using Teensy port: $TEENSY_PORT"

# Stop any existing services
echo "🛑 Stopping existing services..."
TEENSY_PORT="$TEENSY_PORT" sudo -E docker-compose down --remove-orphans || true

# Start the services
echo "🐳 Starting Docker services..."
TEENSY_PORT="$TEENSY_PORT" sudo -E docker-compose up -d

# Wait for services to be ready
echo "⏳ Waiting for services to be ready..."
sleep 5

# Check service status
echo "📊 Service Status:"
TEENSY_PORT="$TEENSY_PORT" sudo -E docker-compose ps

# Show logs
echo "📋 Recent logs:"
echo "--- Micro-ROS Agent ---"
TEENSY_PORT="$TEENSY_PORT" sudo -E docker-compose logs --tail 5 micro-ros-agent

echo "--- ROS2 Workspace ---"
TEENSY_PORT="$TEENSY_PORT" sudo -E docker-compose logs --tail 5 jiggy-joystick-ros2

echo ""
echo "🔄 Resetting Teensy to establish micro-ROS connection..."

# Reset Teensy by toggling DTR (Data Terminal Ready) signal
if command -v python3 >/dev/null 2>&1; then
    python3 -c "
import serial
import time
try:
    ser = serial.Serial('$TEENSY_PORT', 115200, timeout=1)
    ser.dtr = False  # Reset
    time.sleep(0.1)
    ser.dtr = True   # Release reset
    time.sleep(0.5)
    ser.close()
    print('✅ Teensy reset successful')
except Exception as e:
    print(f'⚠️  Teensy reset failed: {e}')
    print('   Manual reset may be required')
"
else
    echo "⚠️  Python3 not found, using stty method..."
    # Alternative method using stty
    sudo stty -F $TEENSY_PORT 1200
    sleep 0.1
    sudo stty -F $TEENSY_PORT 115200
    sleep 0.5
    echo "✅ Teensy reset attempted"
fi

# Wait for micro-ROS connection to establish
echo "⏳ Waiting for micro-ROS connection..."
sleep 5

# Check if connection is established
echo "🔍 Checking micro-ROS connection..."
CONNECTION_CHECK=$(TEENSY_PORT="$TEENSY_PORT" sudo -E docker exec micro-ros-agent /bin/bash -c "source /opt/ros/jazzy/setup.bash \u0026\u0026 ros2 node list 2\u003e/dev/null | grep micro_ros_simulator || echo 'not_found'")

if [ "$CONNECTION_CHECK" != "not_found" ]; then
    echo "✅ Micro-ROS connection established successfully!"
    echo "📋 Available topics:"
    TEENSY_PORT="$TEENSY_PORT" sudo -E docker exec micro-ros-agent /bin/bash -c "source /opt/ros/jazzy/setup.bash \u0026\u0026 ros2 topic list | grep -E '(joint_states|handshake|trial)'"
else
    echo "⚠️  Micro-ROS connection not detected."
    echo ""
    echo "🔌 MANUAL CONNECTION REQUIRED:"
    echo "   The automatic reset didn't work. Please follow these steps:"
    echo ""
    echo "   1. Physically disconnect the Teensy USB cable"
    echo "   2. Wait 2 seconds"
    echo "   3. Reconnect the Teensy USB cable"
    echo "   4. Wait for the Teensy to boot up (LED should blink)"
    echo ""
    echo "   Press ENTER when you have reconnected the Teensy..."
    read -r
    
    echo "⏳ Waiting for Teensy to establish connection..."
    sleep 3
    
    # Check connection again
    CONNECTION_CHECK=$(TEENSY_PORT="$TEENSY_PORT" sudo -E docker exec micro-ros-agent /bin/bash -c "source /opt/ros/jazzy/setup.bash \u0026\u0026 ros2 node list 2\u003e/dev/null | grep micro_ros_simulator || echo 'not_found'")
    
    if [ "$CONNECTION_CHECK" != "not_found" ]; then
        echo "✅ Micro-ROS connection established successfully after manual reconnection!"
        echo "📋 Available topics:"
        TEENSY_PORT="$TEENSY_PORT" sudo -E docker exec micro-ros-agent /bin/bash -c "source /opt/ros/jazzy/setup.bash \u0026\u0026 ros2 topic list | grep -E '(joint_states|handshake|trial)'"
    else
        echo "❌ Connection still not established. Please check:"
        echo "   - Teensy is properly connected to $TEENSY_PORT"
        echo "   - Teensy has the correct micro-ROS firmware"
        echo "   - Device permissions are correct"
        echo "   - Try running: sudo docker logs micro-ros-agent"
    fi
fi

echo ""
echo "🎉 System started successfully!"
echo ""
echo "Next steps:"
echo "  1. Connect to ROS2 container: sudo docker exec -it jiggy-joystick-ros2 bash"
echo "  2. Run the system: ros2 launch robot_orchestrator robot_orchestrator_launch.py"
echo "  3. Monitor topics: ros2 topic list"
echo "  4. View logs: sudo docker-compose logs -f"
echo ""
echo "Teensy port: $TEENSY_PORT"
