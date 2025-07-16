#!/bin/bash

# JiggyJoystick Debug Suite
# Comprehensive debugging toolkit for monitoring ROS2 nodes, topics, and system health

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

# Function to show help
show_help() {
    print_header "🔧 JiggyJoystick Debug Suite"
    echo "=============================================="
    echo "Comprehensive debugging toolkit for monitoring ROS2 nodes, topics, and system health"
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  monitor        - Start real-time system health monitor"
    echo "  debug          - Start interactive topic debugger"
    echo "  record [TIME]  - Record topic data for TIME seconds (default: 30)"
    echo "  analyze        - Analyze experiment logs"
    echo "  quick-check    - Quick system health check"
    echo "  ros-tools      - Run standard ROS2 debugging tools"
    echo "  help           - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 monitor              # Start real-time monitoring"
    echo "  $0 debug                # Start interactive debugging"
    echo "  $0 record 60            # Record data for 60 seconds"
    echo "  $0 analyze              # Analyze logs with plots"
    echo "  $0 quick-check          # Quick system status check"
    echo ""
}

# Function to check if system is running
check_system_status() {
    print_status "Checking JiggyJoystick system status..."
    
    # Check if containers are running
    if ! sudo docker ps | grep -q "jiggy-joystick-ros2"; then
        print_error "JiggyJoystick ROS2 container is not running!"
        echo "Please start the system first with: ./scripts/start_system.sh"
        return 1
    fi
    
    if ! sudo docker ps | grep -q "micro-ros-agent"; then
        print_warning "Micro-ROS agent container is not running!"
        echo "This may be expected if not using the Teensy microcontroller"
    fi
    
    print_status "System containers are running ✓"
    return 0
}

# Function to copy debug tools to container
copy_debug_tools() {
    print_status "Copying debug tools to container..."
    
    # Create debug tools directory in container
    sudo docker exec jiggy-joystick-ros2 mkdir -p /ros2_ws/debug_tools
    
    # Copy all debug tools
    sudo docker cp ./debug_tools/system_monitor.py jiggy-joystick-ros2:/ros2_ws/debug_tools/
    sudo docker cp ./debug_tools/topic_debugger.py jiggy-joystick-ros2:/ros2_ws/debug_tools/
    sudo docker cp ./debug_tools/log_analyzer.py jiggy-joystick-ros2:/ros2_ws/debug_tools/
    
    print_status "Debug tools copied ✓"
}

# Function to install required packages
install_dependencies() {
    print_status "Installing required Python packages..."
    
    sudo docker exec jiggy-joystick-ros2 bash -c "
        pip3 install --break-system-packages psutil matplotlib seaborn pandas numpy
    " || print_warning "Some packages may already be installed"
    
    print_status "Dependencies installed ✓"
}

# Function to run system monitor
run_system_monitor() {
    print_header "🔧 Starting System Monitor"
    echo "=============================="
    
    if ! check_system_status; then
        return 1
    fi
    
    copy_debug_tools
    install_dependencies
    
    print_status "Starting real-time system monitor..."
    print_status "Press Ctrl+C to exit"
    
    sudo docker exec -it jiggy-joystick-ros2 bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        python3 /ros2_ws/debug_tools/system_monitor.py
    "
}

# Function to run topic debugger
run_topic_debugger() {
    print_header "🔧 Starting Topic Debugger"
    echo "=============================="
    
    if ! check_system_status; then
        return 1
    fi
    
    copy_debug_tools
    install_dependencies
    
    print_status "Starting interactive topic debugger..."
    print_status "Use 'help' command inside the debugger for available commands"
    
    sudo docker exec -it jiggy-joystick-ros2 bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        python3 /ros2_ws/debug_tools/topic_debugger.py --interactive
    "
}

# Function to record data
record_data() {
    local record_time=${1:-30}
    
    print_header "🎥 Recording Topic Data"
    echo "=========================="
    
    if ! check_system_status; then
        return 1
    fi
    
    copy_debug_tools
    install_dependencies
    
    print_status "Recording data for ${record_time} seconds..."
    
    sudo docker exec -it jiggy-joystick-ros2 bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        python3 /ros2_ws/debug_tools/topic_debugger.py --record-time ${record_time}
    "
}

# Function to analyze logs
analyze_logs() {
    print_header "📊 Analyzing Experiment Logs"
    echo "=============================="
    
    copy_debug_tools
    install_dependencies
    
    print_status "Analyzing logs in ./logs directory..."
    
    # Run log analyzer on host (since logs are mounted)
    python3 ./debug_tools/log_analyzer.py --log-dir ./logs --plot-trajectories --plot-success --save-plots --export-report
}

# Function to run quick health check
quick_check() {
    print_header "⚡ Quick System Health Check"
    echo "============================"
    
    if ! check_system_status; then
        return 1
    fi
    
    copy_debug_tools
    install_dependencies
    
    print_status "Running quick health check..."
    
    sudo docker exec jiggy-joystick-ros2 bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        echo '🤖 Available ROS2 Nodes:' &&
        ros2 node list &&
        echo '' &&
        echo '📡 Available Topics:' &&
        ros2 topic list &&
        echo '' &&
        echo '🔍 Topic Information:' &&
        ros2 topic info /joint_states &&
        echo '' &&
        echo '📊 Quick Topic Check (5 seconds):' &&
        timeout 5 python3 /ros2_ws/debug_tools/topic_debugger.py || true
    "
}

# Function to run standard ROS2 tools
run_ros_tools() {
    print_header "🛠️ ROS2 Standard Debugging Tools"
    echo "================================="
    
    if ! check_system_status; then
        return 1
    fi
    
    echo "Available ROS2 debugging commands:"
    echo "1. ros2 node list - List all nodes"
    echo "2. ros2 topic list - List all topics"
    echo "3. ros2 topic echo /joint_states - Echo joint states"
    echo "4. ros2 topic hz /joint_states - Check topic frequency"
    echo "5. ros2 node info /control_node - Get node info"
    echo "6. ros2 service list - List all services"
    echo "7. ros2 bag record -a - Record all topics"
    echo ""
    echo "Starting interactive ROS2 shell..."
    
    sudo docker exec -it jiggy-joystick-ros2 bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        echo '🤖 ROS2 Environment Ready!' &&
        echo 'Available commands: ros2 node list, ros2 topic list, ros2 topic echo, etc.' &&
        echo 'Type exit to return to main menu' &&
        bash
    "
}

# Main script logic
main() {
    case "${1:-help}" in
        monitor)
            run_system_monitor
            ;;
        debug)
            run_topic_debugger
            ;;
        record)
            record_data "$2"
            ;;
        analyze)
            analyze_logs
            ;;
        quick-check)
            quick_check
            ;;
        ros-tools)
            run_ros_tools
            ;;
        help|*)
            show_help
            ;;
    esac
}

# Script header
clear
print_header "🔧 JiggyJoystick Debug Suite"
print_header "============================"

# Run main function
main "$@"
