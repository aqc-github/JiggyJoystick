# JiggyJoystick: Robotic Arm Experiment Controller

## Overview

JiggyJoystick is a user-friendly system for running experiments on a 2-degrees-of-freedom (2-DOF) robotic arm. It simplifies robotic arm control using Docker containers and micro-ROS for communication with microcontrollers, designed to support beginners and non-experts.

## Key Features

- **Dynamic Force Fields**: Leverage isotropic, anisotropic, oriented, and time-dependent viscous force fields for comprehensive experimentation
- **Containerization**: Easy to set up and manage experiments using Docker
- **Automated Control and Logging**: Control the robotic arm and log data automatically
- **Dynamic Configuration**: Adjust experiment settings on the fly with ROS 2
- **Micro-ROS Support**: Communicate with microcontrollers (like Teensy 4.1)
- **CSV Data Logging**: Automatic data collection for analysis

## Lessons Learned

- Extensive testing is vital for dynamic systems to prevent unexpected behavior.
- Improvements in automated startup processes enhance reliability.

## Quick Start

### Docker-First Approach

**JiggyJoystick runs entirely in Docker containers for platform-agnostic deployment.** This ensures consistent behavior across all operating systems (Linux, macOS, Windows) and eliminates dependency management issues.

```bash
# Start the entire system with automatic Teensy detection and reset
./scripts/start_system.sh
```

This script will:

- Automatically detect the Teensy device port
- Start the micro-ROS agent container
- Start the ROS2 workspace container, which automatically launches the robot orchestrator
- Reset the Teensy to establish proper communication
- Verify that all topics are available
- **Prompt for manual connection if automatic reset fails**

### Why Docker?

- **Platform Independence**: Works on any system with Docker installed
- **No Local ROS2 Installation**: All ROS2 dependencies are contained within Docker
- **Reproducible Environment**: Consistent behavior across different machines
- **Easy Deployment**: Single command startup with automatic dependency resolution
- **Isolation**: No conflicts with existing software on the host system

### Recent Improvements

We've made several important improvements to the JiggyJoystick system:

- **Fully Automated Startup**: The entire system, including the ROS 2 agent, can now be launched with a single command (`./scripts/start_system.sh`).
- **Enhanced Firmware Robustness**: Updated the Teensy firmware to handle connection failures gracefully. The firmware now includes retry logic that attempts to reconnect every 5 seconds, ensuring robust communication even if the initial connection doesn't succeed.
- **Automatic Teensy Reset**: Integrated an automatic reset feature into the startup script. This triggers a hardware reset of the Teensy after the micro-ROS agent is fully initialized, resolving timing issues and ensuring the XRCE-DDS session is established correctly.
- **Visibility of ROS2 Topics**: Successfully established communication between the Teensy and micro-ROS agent, allowing all expected topics to appear correctly in the ROS2 network.
- **Plug-and-Play Experience**: With the automatic reset and improved connection logic, the system now offers a plug-and-play experience.

## Micro-ROS Integration

The JiggyJoystick system integrates with micro-ROS to communicate with the Teensy 4.1 microcontroller that controls the robotic arm. This integration provides:

### Hardware Communication

- **Teensy 4.1 Node**: `/micro_ros_simulator` publishes joint states and subscribes to control commands
- **Automatic Port Detection**: The system automatically detects the Teensy USB port
- **Robust Connection**: Enhanced firmware with retry logic ensures reliable communication

### Available Topics

Once the system is running, the following topics are available:

| Topic                        | Type                     | Description                              |
| ---------------------------- | ------------------------ | ---------------------------------------- |
| `/joint_states`              | `sensor_msgs/JointState` | Joint positions, velocities, and efforts |
| `/microcontroller_handshake` | `std_msgs/Bool`          | Handshake initiation from Teensy         |
| `/ros2_handshake`            | `std_msgs/Bool`          | Handshake response to Teensy             |
| `/start_trial`               | `std_msgs/Bool`          | Start trial command to Teensy            |
| `/abort_trial`               | `std_msgs/Bool`          | Abort trial command to Teensy            |
| `/trial_success`             | `std_msgs/Bool`          | Trial success notification to Teensy     |

### Monitoring the Connection

To verify the micro-ROS connection is working:

```bash
# Check if the Teensy node is visible
sudo docker exec -it jiggy-joystick-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list"

# Monitor joint states
sudo docker exec -it jiggy-joystick-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic echo /joint_states"

# Check handshake messages
sudo docker exec -it jiggy-joystick-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic echo /microcontroller_handshake"
```

### Firmware Features

The Teensy firmware includes several robust features:

- **Connection Retry Logic**: Automatically retries connection every 5 seconds if initial connection fails
- **Graceful Error Handling**: No longer gets stuck in infinite error loops
- **Handshake Protocol**: Implements a proper handshake sequence before starting trials
- **Trial State Management**: Manages setup, active, and completion states
- **Dynamic Force Field Control**: Applies configurable viscous force fields during trials
  - Isotropic viscous damping
  - Anisotropic viscous damping
  - Oriented viscous damping
  - Time-dependent viscous damping
  - Static force fields (original behavior)

## Experiment Description

The experiment involves running a series of **assays**, each consisting of multiple **trials**, on a 2-DOF robotic arm. Each trial applies specific control parameters (e.g., force fields) for a defined duration, and the system logs joint positions, velocities, torques, and other metadata. The experiment is configured via a YAML file (`assays.yaml`), which specifies:

- The number of assays and trials per assay.
- Trial durations.
- **Dynamic force field settings** with multiple types:
  - Static force fields (original 2x2 matrix)
  - Viscous isotropic (uniform damping)
  - Viscous anisotropic (different X/Y damping)
  - Viscous oriented (rotated damping matrix)
  - Viscous time-dependent (damping changes over time)

The system is designed to:

- Load experiment configurations dynamically.
- Control the robot by applying torques based on joint states and force fields.
- Log data for post-experiment analysis.
- Support dynamic reconfiguration via a ROS 2 service.

### Node Details

1. **ExperimentManagerNode**
   - **Role**: Orchestrates the experiment by iterating through assays and trials defined in `assays.yaml`.
   - **Functionality**:
     - Loads the configuration file using `ament_index_python` to locate `assays.yaml`.
     - Sends trial goals to the `ControlNode` via an action client (`/trial_action`, type `custom_interfaces.action.TrialAction`).
     - Provides a service (`/load_config`, type `custom_interfaces.srv.LoadConfig`) to reload configurations dynamically.
     - Logs trial progress and status.
   - **Key Interactions**:
     - Sends goals to the `ControlNode`’s action server.
     - Receives results (success/aborted) from trials.
     - Handles configuration updates via the service.

2. **ControlNode**
   - **Role**: Executes trials by controlling the robotic arm and computing torques.
   - **Functionality**:
     - Hosts an action server (`/trial_action`) to receive trial goals.
     - Subscribes to `/joint_states` (type `sensor_msgs.msg.JointState`) for joint positions and velocities.
     - Publishes torque commands to `/torque_commands` (type `std_msgs.msg.Float64MultiArray`) at 100 Hz.
     - Publishes trial metadata (e.g., assay/trial numbers, force field status) to topics like `/assay_number`, `/trial_number`, `/ff_enabled`, and `/ff_value`.
     - Computes torques based on joint states and an optional force field matrix.
     - Supports trial cancellation and abort signals.
   - **Key Interactions**:
     - Receives goals from `ExperimentManagerNode`.
     - Publishes data consumed by `LoggerNode`.
     - Requires a hardware interface publishing `/joint_states`.

3. **LoggerNode**
   - **Role**: Logs experiment data to CSV files for analysis.
   - **Functionality**:
     - Subscribes to multiple topics: `/joint_states`, `/torque_commands`, `/assay_number`, `/trial_number`, `/trial_status`, `/ff_enabled`, and `/ff_value`.
     - Creates a new CSV file for each trial, named `log_assayX_trialY_TIMESTAMP.csv`, in the `logs` directory.
     - Logs timestamped data including joint positions, velocities, torques, force field settings, and trial status.
   - **Key Interactions**:
     - Consumes data published by `ControlNode`.
     - Writes to CSV files in the current working directory’s `logs` folder.

## Prerequisites

**JiggyJoystick runs entirely in Docker containers - no local ROS2 installation required!**

### System Requirements

- **Docker**: Installed on your system. See the [official Docker installation guide](https://docs.docker.com/get-docker/).
- **User Groups**: Your user must be in the `docker` and `dialout` groups.
  ```bash
  sudo usermod -aG docker $USER
  sudo usermod -aG dialout $USER
  ```
  (Log out and back in to apply changes).
- **Hardware**: Teensy 4.1 microcontroller with USB connection
- **Operating System**: Any system that supports Docker (Linux, macOS, Windows)

### What's Included in Docker

The Docker containers provide everything needed:

- **ROS 2 Jazzy Jalisco**: Complete ROS2 installation
- **Dependencies**: All ROS 2 packages (`rclpy`, `sensor_msgs`, `std_msgs`, `ament_python`)
- **Python Libraries**: `numpy`, `pyyaml`, and other required packages
- **Custom Interfaces**: `custom_interfaces` package with `TrialAction` and `LoadConfig`
- **Robot Orchestrator**: Complete `robot_orchestrator` package
- **Micro-ROS Agent**: For communication with Teensy microcontroller

## Getting Started

**No installation required!** Simply clone the repository and run the startup script.

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/JiggyJoystick.git
cd JiggyJoystick
```

### 2. Connect Your Teensy

- Connect your Teensy 4.1 to your computer via USB
- Ensure the Teensy has the correct firmware loaded (see firmware documentation)

### 3. Start the System

```bash
./scripts/start_system.sh
```

That's it! The Docker containers will be built automatically and the system will start.

## Standard Operating Procedure (Verified)

Based on extensive testing, the following procedure ensures reliable system operation:

### Quick Start (Recommended)

1. **Connect Teensy**: Ensure Teensy 4.1 is connected via USB
2. **Run startup script**: `./scripts/start_system.sh`
3. **Manual reconnection**: If prompted, physically disconnect and reconnect the Teensy USB cable
4. **Verify connection**: Check that handshake completes successfully

### Manual Docker Control

For advanced users who prefer manual control:

1. **Build containers**:

   ```bash
   sudo docker-compose build --no-cache
   ```

2. **Start services**:

   ```bash
   sudo docker-compose up -d
   ```

3. **Reset Teensy** (if handshake fails):

   ```bash
   python3 -c "
   import serial
   import time
   try:
       ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
       ser.dtr = False  # Reset
       time.sleep(0.1)
       ser.dtr = True   # Release reset
       time.sleep(0.5)
       ser.close()
       print('✅ Teensy reset successful')
   except Exception as e:
       print(f'⚠️  Teensy reset failed: {e}')
   "
   ```

4. **Verify handshake**:

   ```bash
   sudo docker-compose logs jiggy-joystick-ros2 --tail 10
   ```

   Look for "Handshake: ✅ Complete"

5. **Run experiment**:
   ```bash
   sudo docker exec -it jiggy-joystick-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run robot_orchestrator experiment_manager_node"
   ```

### Expected Behavior

- **Initial startup**: System may show "Handshake: ❌ Waiting" for 30-60 seconds
- **After Teensy reset**: Handshake should complete within 5-10 seconds
- **Successful operation**: All trials should be accepted and complete successfully
- **Data flow**: Joint positions and velocities should update in real-time

### Troubleshooting Connection Issues

**If handshake fails:**

1. Check Teensy connection: `ls -la /dev/ttyACM*`
2. Verify micro-ROS node: `sudo docker exec micro-ros-agent bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list"`
3. Look for `/micro_ros_simulator` in the node list
4. Try physical disconnection/reconnection of USB cable
5. Check container logs: `sudo docker-compose logs micro-ros-agent`

**System Status Indicators:**

- ✅ **Handshake Complete**: Ready for experiments
- ✅ **Joint States Receiving**: Real-time data flowing
- ✅ **Action Server Ready**: Can accept trial commands
- ❌ **Handshake Waiting**: Teensy connection needed

## Launch Instructions

To run the `robot_orchestrator` package, simply run the main startup script:

```bash
./scripts/start_system.sh
```

This script handles the entire startup process, including:

- Detecting the Teensy port.
- Launching the `micro-ros-agent` and `jiggy-joystick-ros2` Docker containers.
- Automatically running the `robot_orchestrator_launch.py` file within the `jiggy-joystick-ros2` container.

### Monitoring the System

Once the system is running, you can monitor it in several ways:

- **View Container Logs**:

  ```bash
  sudo docker-compose logs -f
  ```

- **Connect to the ROS 2 Container**:

  ```bash
  sudo docker exec -it jiggy-joystick-ros2 bash
  ```

- **List ROS 2 Topics**:
  ```bash
  ros2 topic list
  ```

## Configuration

The experiment is configured via `assays.yaml` in `~/ros2_ws/src/robot_orchestrator/config/`.

### Dynamic Force Field Configuration

The system now supports multiple types of force fields configured using a new format:

```yaml
assays:
  # Baseline - No force field
  - name: "baseline"
    n_trials: 3
    trial_duration: 10.0
    force_field:
      enabled: false
      matrix: [0.0, 0.0, 0.0, 0.0]

  # Isotropic viscous field
  - name: "viscous_isotropic"
    n_trials: 3
    trial_duration: 10.0
    force_field:
      enabled: true
      matrix: [1, 10.0] # [type_id, damping]

  # Anisotropic viscous field
  - name: "viscous_anisotropic"
    n_trials: 3
    trial_duration: 10.0
    force_field:
      enabled: true
      matrix: [2, 15.0, 5.0] # [type_id, damping_x, damping_y]

  # Time-dependent viscous field
  - name: "viscous_time_dependent"
    n_trials: 3
    trial_duration: 15.0
    force_field:
      enabled: true
      matrix: [4, 0.0, 25.0, 10.0] # [type_id, damping_initial, damping_final, transition_time]
```

### Force Field Types

- **Type 0**: Static force field (original behavior)
- **Type 1**: Viscous isotropic (uniform damping)
- **Type 2**: Viscous anisotropic (different X/Y damping)
- **Type 3**: Viscous oriented (rotated damping matrix)
- **Type 4**: Viscous time-dependent (damping changes over time)

**See `docs/DYNAMIC_FORCE_FIELDS.md` for detailed documentation.**

### Configuration Fields

- **Fields**:
  - `name`: Assay identifier (informational).
  - `n_trials`: Number of trials per assay.
  - `trial_duration`: Duration of each trial in seconds.
  - `force_field.enabled`: Whether to apply a force field.
  - `force_field.matrix`: Force field parameters `[type_id, param1, param2, ...]`.

To reload a new configuration during runtime:

```bash
ros2 service call /load_config custom_interfaces/srv/LoadConfig "{config_file_path: 'new_assays.yaml'}"
```

## Troubleshooting

### General Issues

- **Package Not Found**:
  - Ensure the workspace is built and sourced.
  - Verify `package.xml` and `setup.py` are correct.
- **Action Server Not Available**:
  - Check that `ControlNode` is running (it hosts `/trial_action`).
  - Increase the timeout in `ExperimentManagerNode.run_experiment` (e.g., from 5.0 to 10.0 seconds).
- **No `/joint_states`**:
  - Ensure the robot hardware or a dummy publisher is running.
- **Logging Issues**:
  - Verify `~/ros2_ws/logs` is writable.
  - Check for CSV files after trials complete.
- **Build Errors**:
  - Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`.
  - Check logs in `~/ros2_ws/log/latest_build`.

### Teensy Connection Issues

**Problem**: Micro-ROS agent cannot establish connection with Teensy

**Symptoms**:

- `/micro_ros_simulator` node not visible in `ros2 node list`
- No `/joint_states` topic data
- Micro-ROS agent logs show "Serial port not found" or connection errors

**Solution**: **Manual Teensy Connection Procedure**

1. **Check Teensy Detection**:

   ```bash
   # Verify Teensy is detected by the system
   lsusb | grep -i teensy
   ls -la /dev/ttyACM*
   ```

2. **Manual Reset Process**:
   When the startup script prompts for manual connection:
   - **Physically disconnect** the Teensy USB cable
   - **Wait 2-3 seconds** for the system to detect disconnection
   - **Reconnect** the Teensy USB cable
   - **Wait for bootup** - look for the Teensy LED to blink indicating it's ready
   - **Press ENTER** in the terminal to continue

3. **Verify Connection**:

   ```bash
   # Check if micro-ROS node is now visible
   sudo docker exec jiggy-joystick-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list"

   # Check topics are available
   sudo docker exec jiggy-joystick-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
   ```

4. **If Still Not Working**:
   - Check micro-ROS agent logs: `sudo docker logs micro-ros-agent`
   - Verify Teensy has correct firmware loaded
   - Check USB cable and port functionality
   - Try a different USB port
   - Ensure user has permission to access serial ports (dialout group)

**Note**: This manual connection step is required because the automatic reset mechanism in the startup script doesn't work reliably with all Teensy configurations. The manual disconnect/reconnect ensures the Teensy firmware initializes properly and establishes the micro-ROS connection.

## Example Workflow

1. Launch the system: `./scripts/start_system.sh`.
2. The `ExperimentManagerNode` loads `assays.yaml` (e.g., 3 assays, 2 trials each).
3. For each trial:
   - Sends a goal to `ControlNode` (e.g., assay 1, trial 1, duration 10s, force field enabled).
   - `ControlNode` publishes torques to `/torque_commands` and trial metadata.
   - `LoggerNode` writes data to `logs/log_assay1_trial1_TIMESTAMP.csv`.
4. `ControlNode` completes the trial and sends the result.
5. `ExperimentManagerNode` proceeds to the next trial or assay.
6. After all trials, the experiment completes, and logs are saved.

## Contributing

To contribute:

1. Fork the repository and make your changes
2. Update source files in `ros2_ws/src/robot_orchestrator/` or `ros2_ws/src/custom_interfaces/`
3. Test your changes: `./scripts/start_system.sh` (Docker will automatically rebuild)
4. Submit changes via pull requests

### Development Workflow

- **Code changes**: Edit files in `ros2_ws/src/` - Docker will rebuild automatically
- **Configuration changes**: Modify `ros2_ws/src/robot_orchestrator/config/assays.yaml`
- **Testing**: Use `./scripts/start_system.sh` to test changes
- **Debugging**: Access the container with `sudo docker exec -it jiggy-joystick-ros2 bash`

## License

This package is licensed under the Apache-2.0 License.

