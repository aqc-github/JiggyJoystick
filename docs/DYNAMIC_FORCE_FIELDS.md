# Dynamic Viscous Force Fields

This document explains the implementation and usage of dynamic viscous force fields in the JiggyJoystick system.

## Overview

The system now supports multiple types of force fields beyond the original static matrix approach. Extensive testing and practical application have shown these fields enhance system flexibility and accuracy in controlled scenarios:

1. **Static Force Fields** (original behavior)
2. **Viscous Force Fields** (new implementation)
   - Isotropic viscous
   - Anisotropic viscous
   - Oriented viscous
   - Time-dependent viscous
## Force Field Configuration

Force fields are configured in the `assays.yaml` file using the `matrix` parameter with a new format:

```yaml
force_field:
    enabled: true
    matrix: [type_id, param1, param2, param3, ...]
```

### Type IDs

- `0` - Static force field (original behavior)
- `1` - Viscous isotropic
- `2` - Viscous anisotropic
- `3` - Viscous oriented
- `4` - Viscous time-dependent

## Force Field Types

### 1. Static Force Field (Type ID: 0)

**Description**: Original behavior with constant force matrix.

**Formula**: `F = M * v`
where M is a 2x2 matrix and v is the end-effector velocity.

**Configuration**:
```yaml
matrix: [0, f11, f12, f21, f22]
```

**Example**:
```yaml
force_field:
    enabled: true
    matrix: [0, 5.0, 0.0, 0.0, 5.0]  # Diagonal matrix with 5.0 damping
```

### 2. Viscous Isotropic (Type ID: 1)

**Description**: Uniform damping in all directions.

**Formula**: `F = -b * v`
where b is the damping coefficient.

**Configuration**:
```yaml
matrix: [1, damping]
```

**Example**:
```yaml
force_field:
    enabled: true
    matrix: [1, 10.0]  # 10.0 N⋅s/m uniform damping
```

### 3. Viscous Anisotropic (Type ID: 2)

**Description**: Different damping in X and Y directions.

**Formula**: `F = -diag(bx, by) * v`
where bx and by are damping coefficients for X and Y directions.

**Configuration**:
```yaml
matrix: [2, damping_x, damping_y]
```

**Example**:
```yaml
force_field:
    enabled: true
    matrix: [2, 15.0, 5.0]  # 15.0 N⋅s/m in X, 5.0 N⋅s/m in Y
```

### 4. Viscous Oriented (Type ID: 3)

**Description**: Anisotropic damping rotated by a specified angle.

**Formula**: `F = -R * diag(b_major, b_minor) * R^T * v`
where R is the rotation matrix for the specified angle.

**Configuration**:
```yaml
matrix: [3, damping_major, damping_minor, angle_radians]
```

**Example**:
```yaml
force_field:
    enabled: true
    matrix: [3, 20.0, 5.0, 0.7854]  # Major axis: 20.0, Minor axis: 5.0, Angle: 45°
```

### 5. Viscous Time-Dependent (Type ID: 4)

**Description**: Damping that changes linearly over time.

**Formula**: `F = -b(t) * v`
where `b(t) = b_initial + (b_final - b_initial) * (t / transition_time)` for t ≤ transition_time.

**Configuration**:
```yaml
matrix: [4, damping_initial, damping_final, transition_time]
```

**Example**:
```yaml
force_field:
    enabled: true
    matrix: [4, 0.0, 25.0, 10.0]  # Ramp from 0 to 25 N⋅s/m over 10 seconds
```

## Example Configuration File

```yaml
assays:
    # Baseline - No force field
    - name: "baseline"
      n_trials: 3
      trial_duration: 10.0
      force_field:
          enabled: false
          matrix: [0.0, 0.0, 0.0, 0.0]
    
    # Static force field (original behavior)
    - name: "static_field"
      n_trials: 3
      trial_duration: 10.0
      force_field:
          enabled: true
          matrix: [0, 5.0, 0.0, 0.0, 5.0]  # [type_id, f11, f12, f21, f22]
    
    # Isotropic viscous field
    - name: "viscous_isotropic"
      n_trials: 3
      trial_duration: 10.0
      force_field:
          enabled: true
          matrix: [1, 10.0]  # [type_id, damping]
    
    # Anisotropic viscous field
    - name: "viscous_anisotropic"
      n_trials: 3
      trial_duration: 10.0
      force_field:
          enabled: true
          matrix: [2, 15.0, 5.0]  # [type_id, damping_x, damping_y]
    
    # Oriented viscous field (45 degrees)
    - name: "viscous_oriented"
      n_trials: 3
      trial_duration: 10.0
      force_field:
          enabled: true
          matrix: [3, 20.0, 5.0, 0.7854]  # [type_id, damping_major, damping_minor, angle_rad]
    
    # Time-dependent viscous field
    - name: "viscous_time_dependent"
      n_trials: 3
      trial_duration: 15.0
      force_field:
          enabled: true
          matrix: [4, 0.0, 25.0, 10.0]  # [type_id, damping_initial, damping_final, transition_time]
```

## Implementation Details

### Force Calculation

The force field forces are calculated in the `calculate_dynamic_force_field()` method of the `ControlNode` class. The method:

1. Receives end-effector position and velocity
2. Determines the force field type from `ff_type`
3. Calculates forces based on the type and parameters
4. Returns the force vector

### Torque Conversion

Forces are converted to joint torques using the Jacobian transpose method:
```python
torques = J.T @ ff_forces
```

### Timing

For time-dependent force fields, timing is calculated relative to the trial start time (`ff_start_time`).

## Usage Tips

1. **Start Simple**: Begin with isotropic viscous fields for basic damping effects.

2. **Reasonable Values**: Damping coefficients should be chosen based on your robot's dynamics. Start with values around 1-50 N⋅s/m.

3. **Angle Units**: Angles for oriented fields are in radians. Common values:
   - 0° = 0 radians
   - 45° = 0.7854 radians
   - 90° = 1.5708 radians

4. **Time Dependencies**: Ensure transition times are reasonable compared to trial durations.

5. **Testing**: Use the debug tools to monitor force field behavior in real-time.

## Monitoring

The system provides real-time logging of:
- Force field type and parameters
- Applied forces and torques
- Trial progress and timing

Use the debug suite to monitor force field behavior:
```bash
./debug_tools/debug_suite.sh monitor
```

## Safety Considerations

1. **Magnitude Limits**: Ensure damping coefficients don't create excessive forces that could damage the robot.

2. **Gradual Introduction**: Start with low damping values and gradually increase.

3. **Emergency Stop**: The system supports trial abortion if forces become problematic.

4. **Monitoring**: Always monitor the system during experiments with new force field configurations.

## Future Extensions

The framework can be easily extended to support additional force field types:

- Position-dependent fields
- Velocity-magnitude dependent fields
- Adaptive fields based on performance
- Stochastic/random perturbations

To add new types, modify the `calculate_dynamic_force_field()` method and add corresponding parsing logic in `parse_force_field_config()`.
