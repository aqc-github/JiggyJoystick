# JiggyJoystick Experiment Analysis

This document describes the automatic post-experiment analysis functionality that has been added to the JiggyJoystick system.

## Overview

After each experiment completes, the system automatically:
1. **Analyzes all logged data** from the experiment trials
2. **Generates comprehensive visualizations** of robot performance
3. **Produces statistical summaries** of the experimental results
4. **Saves high-resolution plots** for documentation and further analysis

## Features

### 📊 Automated Analysis Pipeline
- **Automatic Execution**: Analysis runs immediately after experiment completion
- **Comprehensive Coverage**: Analyzes all assays and trials in a single run
- **Fast Processing**: Typically completes in under 2 seconds
- **Error Recovery**: Robust handling of malformed or incomplete data

### 📈 Visualization Capabilities
- **Joint Analysis**: Position, velocity, and effort plots over time for all joints
- **End-Effector Trajectories**: 2D workspace trajectories reconstructed from joint data
- **Multi-Assay Comparison**: Overlaid plots comparing performance across different assays
- **Statistical Dashboard**: Comprehensive performance metrics and workspace analysis

### 🔢 Statistical Reporting
- Experiment duration and sampling rate analysis
- Joint position and velocity range characterization
- End-effector workspace calculation
- Performance consistency metrics across trials

## Usage

### Automatic Execution
The analysis runs automatically after experiments complete. No manual intervention required.

### Manual Execution
You can also run the analysis manually on existing log data:

```bash
# Inside the Docker container
python3 /ros2_ws/experiment_analyzer.py

# With specific options
python3 /ros2_ws/experiment_analyzer.py --no-show --log-dir /custom/path
```

### Command Line Options
- `--log-dir`: Specify custom log directory (default: `/ros2_ws/logs`)
- `--no-show`: Disable interactive plot display (useful for headless execution)
- `--no-save`: Skip saving plots to files

## Output Files

### Generated Plots
All plots are saved to the `analysis_output/` directory with timestamps:
- `joint_analysis_YYYYMMDD_HHMMSS.png`: Joint position, velocity, and effort analysis
- `end_effector_analysis_YYYYMMDD_HHMMSS.png`: End-effector trajectory and position analysis

### Log Analysis
The analyzer processes CSV log files with the following structure:
```csv
timestamp,joint_positions,joint_velocities,joint_efforts
2025-07-25 17:04:03.753,"[-0.145, -0.346]","[-0.239, 0.108]","[0.0, 0.0]"
```

## Analysis Details

### Forward Kinematics Reconstruction
The system reconstructs end-effector positions from joint angles using the 5-bar parallel robot kinematics:
- **Link Lengths**: L1 = 63.5mm, L2 = 63.5mm, d = 50.8mm (base width)
- **Coordinate System**: Standard Cartesian coordinates in meters
- **Approximation**: Uses midpoint method for simplified circle intersection

### Statistical Metrics
- **Duration**: Total time from first to last data point
- **Sample Rate**: Average data collection frequency (Hz)
- **Position Range**: Min/max joint angles in radians
- **Velocity Analysis**: Maximum angular velocities
- **Workspace Area**: End-effector reachable area in mm²

### Multi-Assay Analysis
- **Color Coding**: Each assay gets a unique color for easy identification
- **Legend Support**: Comprehensive legends for all plots
- **Overlay Visualization**: Multiple assays displayed simultaneously for comparison

## Integration with Experiment Manager

The analysis is integrated into the `ExperimentManagerNode` via the `run_post_experiment_analysis()` method:

1. **Automatic Trigger**: Called at the end of `run_experiment()`
2. **Subprocess Execution**: Runs analyzer as a separate Python process
3. **Timeout Protection**: 60-second timeout to prevent hanging
4. **Error Handling**: Comprehensive error reporting in ROS2 logs
5. **Output Logging**: Key analysis results logged to ROS2 system

## Testing

Use the provided test script to verify the analysis workflow:

```bash
# Inside Docker container
python3 /ros2_ws/test_analysis_workflow.py
```

This test:
- ✅ Verifies analyzer script availability
- ✅ Checks for existing log data
- ✅ Runs complete analysis pipeline
- ✅ Validates output file generation
- ✅ Reports performance metrics

## Example Output

```
📊 EXPERIMENT STATISTICS REPORT
================================================================================

Assay 1:
  Duration: 7.93 seconds
  Data Points: 80
  Average Sample Rate: 10.09 Hz
  Joint 1 Position Range: -0.500 to 0.450 rad
  Joint 2 Position Range: -0.346 to 0.500 rad
  Max Joint 1 Velocity: 0.250 rad/s
  Max Joint 2 Velocity: 0.150 rad/s
  End-Effector Workspace: 227.90 mm²

Assay 2:
  Duration: 7.93 seconds
  Data Points: 80
  Average Sample Rate: 10.09 Hz
  Joint 1 Position Range: -0.500 to 0.498 rad
  Joint 2 Position Range: -0.463 to 0.465 rad
  Max Joint 1 Velocity: 0.250 rad/s
  Max Joint 2 Velocity: 0.150 rad/s
  End-Effector Workspace: 384.16 mm²
```

## Technical Implementation

### Key Components
- **`JiggyJoystickAnalyzer`**: Main analysis class
- **`experiment_analyzer.py`**: Standalone analysis script
- **`run_post_experiment_analysis()`**: Integration method in ExperimentManagerNode

### Dependencies
- **pandas**: Data manipulation and CSV processing
- **matplotlib**: Plot generation and visualization
- **numpy**: Numerical computations and kinematics
- **glob**: File pattern matching for log discovery

### Performance
- **Processing Speed**: ~1.5 seconds for 12 assays with 80-120 data points each
- **Memory Usage**: Minimal - processes files sequentially
- **File Size**: Generated plots are ~300KB each in high resolution (300 DPI)

## Future Enhancements

Potential improvements for the analysis system:
1. **Interactive Dashboards**: Web-based real-time analysis interface
2. **Email Reports**: Automatic email delivery of analysis results
3. **Comparative Analysis**: Historical performance comparison across experiments
4. **Advanced Statistics**: Statistical significance testing and trend analysis
5. **Export Formats**: Additional output formats (PDF reports, Excel spreadsheets)
6. **Real-time Analysis**: Live analysis during experiment execution

## Troubleshooting

### Common Issues
1. **Missing Dependencies**: Ensure pandas, matplotlib, and numpy are installed
2. **No Log Files**: Verify experiments have completed and generated logs
3. **Permission Errors**: Check file permissions in the analysis_output directory
4. **Timeout Errors**: For very large datasets, increase the timeout in the integration code

### Debug Mode
For detailed debugging, run the analyzer directly:
```bash
python3 /ros2_ws/experiment_analyzer.py --log-dir /ros2_ws/logs
```

This provides verbose output and error details not available through the integration layer.
