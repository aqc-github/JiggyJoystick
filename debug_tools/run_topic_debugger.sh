#!/bin/bash

# JiggyJoystick Topic Debugger Runner
# This script helps run the topic debugger inside the Docker container

set -e

echo "🔧 Starting JiggyJoystick Topic Debugger"
echo "========================================"

# Check if we're running inside Docker
if [ -f /.dockerenv ]; then
    echo "⚙️ Running inside Docker container"
    
    # Source ROS2 environment
    source /opt/ros/jazzy/setup.bash
    source /ros2_ws/install/setup.bash
    
    # Run the topic debugger
    python3 /ros2_ws/debug_tools/topic_debugger.py
else
    echo "🛠️ Running outside Docker container"
    
    # Check if the ROS2 container is running
    if ! sudo docker ps | grep -q "jiggy-joystick-ros2"; then
        echo "❌ JiggyJoystick ROS2 container is not running!"
        echo "Please start the system first with: ./scripts/start_system.sh"
        exit 1
    fi
    
    # Copy debug tools to container
    sudo docker cp ./debug_tools/system_monitor.py jiggy-joystick-ros2:/ros2_ws/debug_tools/
    sudo docker cp ./debug_tools/topic_debugger.py jiggy-joystick-ros2:/ros2_ws/debug_tools/
    sudo docker cp ./debug_tools/log_analyzer.py jiggy-joystick-ros2:/ros2_ws/debug_tools/
    
    # Run the topic debugger inside the container
    sudo docker exec -it jiggy-joystick-ros2 bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        mkdir -p /ros2_ws/debug_tools &&
        python3 /ros2_ws/debug_tools/topic_debugger.py
    "
fi

