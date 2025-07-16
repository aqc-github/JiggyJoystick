# JiggyJoystick Debug Tools

Comprehensive debugging and monitoring toolkit for the JiggyJoystick robotic arm experiment system.

## Overview

This toolkit provides a complete set of debugging tools to monitor, analyze, and troubleshoot your JiggyJoystick ROS2 system. The tools are designed to help you understand the behavior of your robotic arm experiments and diagnose issues quickly.

## Quick Start

### 1. Use the Debug Suite (Recommended)

The easiest way to access all debugging tools is through the main debug suite:

```bash
./debug_tools/debug_suite.sh
```

This will show you all available options:
- `monitor` - Real-time system health monitoring
- `debug` - Interactive topic debugging
- `record` - Record topic data for analysis
- `analyze` - Analyze experiment logs
- `quick-check` - Quick system health check
- `ros-tools` - Standard ROS2 debugging tools

### 2. Examples

```bash
# Start real-time monitoring
./debug_tools/debug_suite.sh monitor

# Interactive debugging session
./debug_tools/debug_suite.sh debug

# Record data for 60 seconds
./debug_tools/debug_suite.sh record 60

# Analyze existing logs
./debug_tools/debug_suite.sh analyze

# Quick system check
./debug_tools/debug_suite.sh quick-check
```

## Individual Tools

### 1. System Monitor (`system_monitor.py`)

Real-time monitoring dashboard that shows:
- System resource usage (CPU, memory, disk)
- ROS2 node health status
- Topic health and message frequencies
- Current experiment status
- Joint position and velocity data

**Features:**
- Live updating display
- Health status indicators
- Frequency monitoring
- Resource usage tracking

**Usage:**
```bash
# Direct usage (inside container)
python3 system_monitor.py

# Using debug suite
./debug_tools/debug_suite.sh monitor
```

### 2. Topic Debugger (`topic_debugger.py`)

Interactive tool for inspecting and analyzing ROS2 topics:
- Record topic data
- Export to CSV/JSON
- Generate plots
- Real-time topic statistics
- Interactive command interface

**Features:**
- Record/playback functionality
- Data export capabilities
- Plotting and visualization
- Statistical analysis
- Interactive commands

**Usage:**
```bash
# Interactive mode
python3 topic_debugger.py --interactive

# Auto-record for 30 seconds
python3 topic_debugger.py --record-time 30

# Monitor specific topics
python3 topic_debugger.py --topics /joint_states /torque_commands

# Using debug suite
./debug_tools/debug_suite.sh debug
```

**Interactive Commands:**
- `status` - Show topic status
- `record` - Start recording
- `stop` - Stop recording
- `save [filename]` - Save recorded data
- `export <topic> [filename]` - Export topic to CSV
- `plot <topic> [field]` - Plot topic data
- `stats <topic>` - Show topic statistics
- `list` - List monitored topics
- `quit` - Exit

### 3. Log Analyzer (`log_analyzer.py`)

Analyze experiment logs and generate insights:
- Load and parse CSV log files
- Generate summary statistics
- Create visualizations
- Export analysis reports
- Joint trajectory analysis

**Features:**
- Automatic log discovery
- Statistical analysis
- Trajectory plotting
- Success rate analysis
- Comprehensive reporting

**Usage:**
```bash
# Analyze logs with plots
python3 log_analyzer.py --plot-trajectories --plot-success --save-plots

# Export analysis report
python3 log_analyzer.py --export-report

# Analyze specific assay/trial
python3 log_analyzer.py --assay 1 --trial 2 --plot-trajectories

# Using debug suite
./debug_tools/debug_suite.sh analyze
```

## Monitored Topics

The debugging tools monitor these JiggyJoystick topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions, velocities, efforts |
| `/torque_commands` | `std_msgs/Float64MultiArray` | Torque commands to joints |
| `/assay_number` | `std_msgs/Int32` | Current assay number |
| `/trial_number` | `std_msgs/Int32` | Current trial number |
| `/trial_status` | `std_msgs/Int32` | Trial status (0=setup, 1=active, -1=abort, 2=complete) |
| `/trial_success` | `std_msgs/Bool` | Trial success indicator |
| `/start_trial` | `std_msgs/Bool` | Trial start command |
| `/abort_trial` | `std_msgs/Bool` | Trial abort command |
| `/ros2_handshake` | `std_msgs/Bool` | ROS2 handshake signal |
| `/microcontroller_handshake` | `std_msgs/Bool` | Microcontroller handshake signal |

## Installation

The debug tools require additional Python packages. They are automatically installed when using the debug suite, but you can install them manually:

```bash
pip3 install psutil matplotlib seaborn pandas numpy
```

## File Structure

```
debug_tools/
├── README.md                 # This documentation
├── debug_suite.sh           # Main debug suite launcher
├── system_monitor.py        # Real-time system monitor
├── topic_debugger.py        # Interactive topic debugger
├── log_analyzer.py          # Log analysis tool
├── run_system_monitor.sh    # System monitor runner
└── run_topic_debugger.sh    # Topic debugger runner
```

## Troubleshooting

### Common Issues

1. **Container Not Running**
   - Ensure JiggyJoystick system is started: `./scripts/start_system.sh`
   - Check container status: `sudo docker ps`

2. **Missing Dependencies**
   - The debug suite automatically installs required packages
   - For manual installation: `pip3 install psutil matplotlib seaborn pandas numpy`

3. **No Topic Data**
   - Check if nodes are running: `ros2 node list`
   - Verify topic publishing: `ros2 topic hz /joint_states`
   - Check handshake status between ROS2 and microcontroller

4. **Permission Issues**
   - Ensure user is in docker group: `sudo usermod -aG docker $USER`
   - Log out and back in after adding to group

### Debug Workflow

1. **Start with Quick Check**
   ```bash
   ./debug_tools/debug_suite.sh quick-check
   ```

2. **Use System Monitor for Real-time Issues**
   ```bash
   ./debug_tools/debug_suite.sh monitor
   ```

3. **Record Data for Analysis**
   ```bash
   ./debug_tools/debug_suite.sh record 60
   ```

4. **Analyze Logs After Experiments**
   ```bash
   ./debug_tools/debug_suite.sh analyze
   ```

5. **Use Interactive Debugger for Deep Inspection**
   ```bash
   ./debug_tools/debug_suite.sh debug
   ```

## Integration with JiggyJoystick

The debug tools are designed to work seamlessly with the JiggyJoystick system:

1. **Automatic Container Detection** - Tools detect if system is running
2. **Log Integration** - Analyzes logs in the standard `./logs` directory
3. **Topic Compatibility** - Monitors all JiggyJoystick-specific topics
4. **Docker Integration** - Automatically runs inside the ROS2 container
5. **Non-intrusive** - Monitoring doesn't affect experiment performance

## Output Files

The tools generate various output files:

- `topic_debug_TIMESTAMP.json` - Recorded topic data
- `topic_debug_TOPIC_TIMESTAMP.csv` - Exported topic data
- `topic_plot_TOPIC_TIMESTAMP.png` - Topic visualizations
- `joint_trajectories_TIMESTAMP.png` - Joint trajectory plots
- `trial_success_analysis_TIMESTAMP.png` - Success analysis plots
- `experiment_analysis_TIMESTAMP.json` - Comprehensive analysis report

## Best Practices

1. **Start Monitoring Early** - Begin monitoring before starting experiments
2. **Record Key Sessions** - Record data during important experimental runs
3. **Regular Health Checks** - Use quick-check to verify system health
4. **Analyze Logs Regularly** - Review experiment logs after each session
5. **Save Debug Data** - Keep recorded data for later analysis
6. **Use Interactive Mode** - Leverage interactive debugger for complex issues

## Contributing

To add new debugging features:

1. Create new Python scripts in the `debug_tools/` directory
2. Follow the existing patterns for Docker integration
3. Add new options to `debug_suite.sh`
4. Update this documentation
5. Test with the existing JiggyJoystick system

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review the main JiggyJoystick documentation
3. Use the interactive debugger to inspect system state
4. Collect debug data using the recording tools
