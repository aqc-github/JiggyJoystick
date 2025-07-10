# Robot Orchestrator ROS 2 Package

## Overview

The `robot_orchestrator` package is a ROS 2 package designed to manage and execute experiments for a 2-DOF (degrees of freedom) robotic arm. It orchestrates trials defined in a YAML configuration file, controls the robot by computing and publishing torque commands, and logs experiment data to CSV files for analysis. The package is built for ROS 2 Jazzy Jalisco and relies on a custom interfaces package (`custom_interfaces`) for action and service definitions.

The system consists of three nodes:
- **ExperimentManagerNode**: Orchestrates experiments by loading configurations and sending trial goals to the control node.
- **ControlNode**: Executes trials by controlling the robotic arm, computing torques, and providing feedback.
- **LoggerNode**: Logs trial data (joint states, torques, etc.) to CSV files.

The package is launched using a single launch file, `robot_orchestrator_launch.py`, which starts all three nodes.

## Experiment Description

The experiment involves running a series of **assays**, each consisting of multiple **trials**, on a 2-DOF robotic arm. Each trial applies specific control parameters (e.g., force fields) for a defined duration, and the system logs joint positions, velocities, torques, and other metadata. The experiment is configured via a YAML file (`assays.yaml`), which specifies:
- The number of assays and trials per assay.
- Trial durations.
- Force field settings (enabled/disabled and a 2x2 matrix for force calculations).

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

To deploy and run the `robot_orchestrator` package, ensure the following:

- **ROS 2 Jazzy Jalisco**: Installed on your system (e.g., Raspberry Pi with Ubuntu 24.04).
- **Dependencies**:
  - ROS 2 packages: `rclpy`, `sensor_msgs`, `std_msgs`, `ament_python`.
  - Python libraries: `numpy`, `pyyaml`.
  - Custom ROS 2 package: `custom_interfaces` (defines `TrialAction` and `LoadConfig`).
- **Robot Hardware**: A 2-DOF robotic arm with a ROS 2 node publishing `/joint_states` (type `sensor_msgs.msg.JointState`) and subscribing to `/torque_commands` (type `std_msgs.msg.Float64MultiArray`).
- **Workspace**: A ROS 2 workspace containing `robot_orchestrator` and `custom_interfaces` packages.

## Installation

Follow these steps to set up the package on a Raspberry Pi (or similar system) running Ubuntu 24.04 with ROS 2 Jazzy Jalisco.

### 1. Install ROS 2 Jazzy
```bash
sudo apt update && sudo apt upgrade
sudo apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 2. Install Build Tools and Dependencies
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
pip3 install numpy pyyaml
```

### 3. Set Up the ROS 2 Workspace
Create a workspace and clone the required packages:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone or copy robot_orchestrator and custom_interfaces packages
# Example: scp -r robot_orchestrator custom_interfaces user@host:~/ros2_ws/src/
```

Ensure the workspace structure is:
```
~/ros2_ws/
└── src/
    ├── custom_interfaces/
    │   ├── action/TrialAction.action
    │   ├── srv/LoadConfig.srv
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── robot_orchestrator/
        ├── config/assays.yaml
        ├── launch/robot_orchestrator_launch.py
        ├── robot_orchestrator/main.py
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

### 4. Build the Workspace
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 5. Set Up Logging Directory
Create a writable directory for log files:
```bash
mkdir -p ~/ros2_ws/logs
chmod 777 ~/ros2_ws/logs
```

### 6. Configure Robot Hardware
Ensure your robotic arm’s ROS 2 driver is installed and configured to:
- Publish `/joint_states` (type `sensor_msgs.msg.JointState`) with joint positions and velocities for two joints (`joint1`, `joint2`).
- Subscribe to `/torque_commands` (type `std_msgs.msg.Float64MultiArray`) to apply torque commands.
Example (for testing without hardware):
```bash
ros2 topic pub /joint_states sensor_msgs/JointState "header: {frame_id: ''}, name: ['joint1', 'joint2'], position: [0.0, 0.0], velocity: [0.0, 0.0], effort: [0.0, 0.0]" -r 10
```

## Launch Instructions

To run the `robot_orchestrator` package:
1. **Source the Workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Launch the Package**:
   ```bash
   ros2 launch robot_orchestrator robot_orchestrator_launch.py
   ```
   This starts:
   - `experiment_manager_node`: Loads `assays.yaml` and manages trials.
   - `control_node`: Controls the robot and executes trials.
   - `logger_node`: Logs data to CSV files in `~/ros2_ws/logs`.

3. **Monitor Output**:
   - Check console output for node initialization and trial progress.
   - Verify CSV files in `~/ros2_ws/logs` (e.g., `log_assay1_trial1_TIMESTAMP.csv`).
   - List topics to confirm communication:
     ```bash
     ros2 topic list
     ```
     Expected topics: `/joint_states`, `/torque_commands`, `/assay_number`, `/trial_number`, `/trial_status`, `/ff_enabled`, `/ff_value`, `/abort_trial`.

4. **Stop the Experiment**:
   Press `Ctrl+C` to stop the nodes. The `LoggerNode` will close any open CSV files.

## Configuration

The experiment is configured via `assays.yaml` in `~/ros2_ws/src/robot_orchestrator/config/`. Example:
```yaml
assays:
  - name: assay1
    n_trials: 2
    trial_duration: 10.0
    force_field:
      enabled: true
      matrix: [1.0, 0.0, 0.0, 1.0]
  - name: assay2
    n_trials: 2
    trial_duration: 10.0
    force_field:
      enabled: false
      matrix: [0.0, 0.0, 0.0, 0.0]
  - name: assay3
    n_trials: 2
    trial_duration: 10.0
    force_field:
      enabled: true
      matrix: [0.5, 0.0, 0.0, 0.5]
```

- **Fields**:
  - `name`: Assay identifier (informational).
  - `n_trials`: Number of trials per assay.
  - `trial_duration`: Duration of each trial in seconds.
  - `force_field.enabled`: Whether to apply a force field.
  - `force_field.matrix`: 2x2 matrix for force calculations (flattened to a list).

To reload a new configuration during runtime:
```bash
ros2 service call /load_config custom_interfaces/srv/LoadConfig "{config_file_path: 'new_assays.yaml'}"
```

## Troubleshooting

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

## Example Workflow

1. Launch the system: `ros2 launch robot_orchestrator robot_orchestrator_launch.py`.
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
1. Fork the repository or modify `~/ros2_ws/src/robot_orchestrator`.
2. Update `main.py`, `assays.yaml`, or other files.
3. Rebuild and test: `colcon build && ros2 launch robot_orchestrator robot_orchestrator_launch.py`.
4. Submit changes via pull requests or update the workspace.

## License

This package is licensed under the Apache-2.0 License.