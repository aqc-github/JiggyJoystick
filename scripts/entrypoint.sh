#!/bin/bash

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Check if workspace is built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    echo "Sourcing ROS2 workspace..."
    source /ros2_ws/install/setup.bash
else
    echo "ROS2 workspace not built, building now..."
    cd /ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source /ros2_ws/install/setup.bash
fi

# Set up environment
export ROS_DOMAIN_ID=0
export PYTHONPATH=/ros2_ws/install/lib/python3.12/site-packages:$PYTHONPATH

# Create logs directory if it doesn't exist
mkdir -p /ros2_ws/logs

echo "ROS2 environment ready!"
echo "Available ROS2 packages:"
ros2 pkg list | grep -E "(robot_orchestrator|custom_interfaces)" || echo "No custom packages found"

echo "To run the system:"
echo "  ros2 launch robot_orchestrator robot_orchestrator_launch.py"
echo ""
echo "To check topics:"
echo "  ros2 topic list"
echo ""
echo "To monitor joint states:"
echo "  ros2 topic echo /joint_states"

# Execute the provided command
exec "$@"
