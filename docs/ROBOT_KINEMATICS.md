# 5-Bar Parallel Robot Kinematics

This document explains the kinematics of a 5-bar parallel robot, including direct and inverse kinematic formulas, the coordinate system definition, and the rationale behind the chosen link lengths (\( l_a = 20 \, \text{mm} \), \( l_b = 40 \, \text{mm} \), \( l_c = 50 \, \text{mm} \)) to achieve a 40mm x 30mm operating area inscribed within the workspace.

## Overview

The 5-bar parallel robot is a 2-DOF mechanism designed for high rigidity, accuracy, and speed compared to serial manipulators. It consists of two active links (length \( l_a \)) connected to fixed joints at O and B, two passive links (length \( l_b \)), and an end-effector at point P, forming two serial chains (O-A-P and B-C-P) that converge at P. The kinematics are critical for determining the end-effector's position from joint angles (direct kinematics) and computing joint angles from the end-effector's position (inverse kinematics). The design choices ensure a workspace that can inscribe a specified 40mm x 30mm operating area while adhering to mechanical constraints.

## Coordinate System Definition

The global coordinate system is defined as follows:

- **Origin**: Located at fixed joint O, i.e., \( O = (0, 0) \, \text{mm} \).
- **Fixed Joint B**: Located at \( (l_c, 0) \, \text{mm} \), where \( l_c \) is the base link length (distance between fixed joints O and B).
- **Axes**: The X-axis is horizontal, aligned along the line from O to B. The Y-axis is vertical, perpendicular to the X-axis, forming a right-handed Cartesian coordinate system.

**Mathematical Basis**:

- The placement of O at \( (0, 0) \) simplifies kinematic equations by setting one fixed joint at the origin.
- The base link length \( l_c \) defines the separation between O and B, placed along the X-axis for symmetry, as the workspace is symmetric about \( x = l_c/2 \).
- Joint A is at \( (l_a \cos \theta_1, l_a \sin \theta_1) \), and joint C is at \( (l_c + l_a \cos \theta_2, l_a \sin \theta_2) \), where \( \theta_1 \) and \( \theta_2 \) are the angles of the active links relative to the positive X-axis.

## Kinematic Formulas

### Direct Kinematics

**Description**: Computes the end-effector position \( P = (x_P, y_P) \) given the joint angles \( \theta_1 \) and \( \theta_2 \).

**Formula**: Based on the paper's forward kinematic equations (Section 2.1), the position of P is derived by solving the intersection of the two serial chains. The coordinates are:

\[
\begin{cases}
x_P = l_a \cos \theta_1 + l_b \cos \theta_3 \\
y_P = l_a \sin \theta_1 + l_b \sin \theta_3
\end{cases}
\]

where \( \theta_3 \) is the angle of the passive link A-P, determined by solving the constraint from the right chain B-C-P. The paper provides a simplified form (Eqn. 1):

\[
\begin{cases}
x_P = l_a \cos \theta_1 + l_b \cos \left[ 2 \arctan \left( \frac{-F + \sqrt{F^2 + G^2}}{G - E} \right) \right] \\
y_P = l_a \sin \theta_1 + l_b \sin \left[ 2 \arctan \left( \frac{-F + \sqrt{F^2 + G^2}}{G - E} \square \right) \right]
\end{cases}
\]

where:

- \( E = 2 l_a (1 - (\cos \theta_1 - \cos \theta_2)^2) \)
- \( F = 2 l_a (\sin \theta_1 - \sin \theta_2) \)
- \( G = l_a^2 + 2 l_b^2 + 2 l_a l_b (\cos \theta_1 - \cos \theta_2) - l_c^2 \)

**Implementation Notes**:

- The discriminant \( F^2 + G^2 \) must be non-negative for a real solution.
- The equations account for the geometric constraints of the closed-loop structure.

### Inverse Kinematics

**Description**: Computes the joint angles \( \theta_1 \) and \( \theta_2 \) given the end-effector position \( P = (x, y) \).

**Formula**: For point P at \( (x, y) \), the distances from O and B are:

- \( d\_{O-P} = \sqrt{x^2 + y^2} \)
- \( d\_{B-P} = \sqrt{(x - l_c)^2 + y^2} \)

The angles are calculated using the law of cosines for each chain:

- **Left chain (O-A-P)**:
  \[
  \cos \alpha = \frac{l*a^2 + d*{O-P}^2 - l*b^2}{2 l_a d*{O-P}}
  \]
  \[
  \theta_1 = \phi \pm \alpha, \quad \phi = \arctan2(y, x)
  \]

- **Right chain (B-C-P)**:
  \[
  \cos \beta = \frac{l*a^2 + d*{B-P}^2 - l*b^2}{2 l_a d*{B-P}}
  \]
  \[
  \theta_2 = \psi \pm \beta, \quad \psi = \arctan2(y, x - l_c)
  \]

**Implementation Notes**:

- Two solutions exist per chain (\( \pm \alpha \), \( \pm \beta \)). Typically, one solution is chosen based on configuration (e.g., \( \theta_1 = \phi - \alpha \), \( \theta_2 = \psi + \beta \)).
- Ensure \( |\cos \alpha| \leq 1 \) and \( |\cos \beta| \leq 1 \) for feasible positions.
- Angles are normalized to \( [0, 2\pi) \).

## Link Length Selection

The link lengths were chosen as \( l_a = 20 \, \text{mm} \) (proximal), \( l_b = 40 \, \text{mm} \) (distal), and \( l_c = 50 \, \text{mm} \) (base) based on the following design criteria from the paper:

1. **Avoiding Interference (Criteria 1)**:
   - **Requirement**: \( l_c = 2 l_a + k \), where \( k \) is a safety distance to prevent active link collision.
   - **Choice**: Set \( k = 10 \, \text{mm} \) (a reasonable safety margin). Thus, \( l_c = 2 \times 20 + 10 = 50 \, \text{mm} \).
   - **Rationale**: Ensures the active links can fully rotate without interference, as per Eqn. (8) in the paper.

2. **Eliminating Singularities (Criteria 2)**:
   - **Requirement**: Proper link length ratios to avoid Type II and Type III singularities (Section 3.2).
   - **Choice**: \( l_b = 40 \, \text{mm} > l_a = 20 \, \text{mm} \).
   - **Rationale**: \( l_b > l_a \) reduces singularity occurrences within the workspace, ensuring stable operation. The paper notes that Type I singularities occur at boundaries, which are acceptable if the operating area is interior.

3. **Workspace Containing 40mm x 30mm Operating Area**:
   - **Requirement**: The workspace, defined as the intersection of two annular regions (radii \( |l_a - l_b| \) to \( l_a + l_b \)), must inscribe a 40mm x 30mm rectangle.
   - **Choice**: \( l_a = 20 \, \text{mm} \), \( l_b = 40 \, \text{mm} \), \( l_c = 50 \, \text{mm} \).
   - **Rationale**:
     - **Workspace Size**: The reachable distance from O or B ranges from \( |20 - 40| = 20 \, \text{mm} \) to \( 20 + 40 = 60 \, \text{mm} \). The workspace is a lens-shaped region symmetric about \( x = l_c/2 = 25 \, \text{mm} \).
     - **Operating Area Fit**: A 40mm x 30mm rectangle centered at \( (25, 15) \, \text{mm} \) (y-offset to avoid boundary singularities) has vertices within the workspace. For example, vertex at \( (45, 30) \):
       - Distance from O: \( \sqrt{45^2 + 30^2} \approx 54.08 \, \text{mm} \), which is between 20 and 60 mm.
       - Distance from B: \( \sqrt{(45 - 50)^2 + 30^2} \approx 30.41 \, \text{mm} \), also between 20 and 60 mm.
     - All vertices satisfy \( |l_a - l_b| \leq d \leq l_a + l_b \), confirming the rectangle fits.

**Additional Considerations**:

- The workspace width at \( x = l_c/2 = 25 \, \text{mm} \) is sufficient (approximately 40 mm, as \( 2 \times (l_a + l_b - l_c/2) = 2 \times (20 + 40 - 25) = 70 \, \text{mm} \)).
- The y-extent at \( x = 25 \, \text{mm} \) is \( \sqrt{(20 + 40)^2 - 25^2} \approx 54.77 \, \text{mm} \), accommodating the 30mm height.
- These lengths balance compactness (small \( l_a \)) with workspace size (larger \( l_b \)) while meeting the safety margin \( k = 10 \, \text{mm} \).

## Implementation Details

### Kinematic Calculations

The kinematic formulas are implemented in a control system as follows:

- **Direct Kinematics**: Compute \( x_P, y_P \) using the forward kinematic equations, typically in a `compute_end_effector_position()` function.
- **Inverse Kinematics**: Solve for \( \theta_1, \theta_2 \) using the inverse kinematic equations, implemented in a `compute_joint_angles()` function.
- **Validation**: Ensure \( |\cos \alpha| \leq 1 \) and \( |\cos \beta| \leq 1 \) to verify the target position is reachable.

### Workspace Analysis

The workspace is computed as the intersection of two annular regions:

- Left chain (O-A-P): Points P where \( |l_a - l_b| \leq \sqrt{x^2 + y^2} \leq l_a + l_b \).
- Right chain (B-C-P): Points P where \( |l_a - l_b| \leq \sqrt{(x - l_c)^2 + y^2} \leq l_a + l_b \).

This is typically implemented by sampling points or using geometric constraints in simulation software.

## Usage Tips

1. **Coordinate System**: Always reference positions relative to O at \( (0, 0) \). Ensure joint B is correctly positioned at \( (l_c, 0) \).
2. **Angle Units**: All angles (\( \theta_1, \theta_2, \phi, \psi, \alpha, \beta \)) are in radians.
3. **Singularity Avoidance**: Operate the end-effector away from workspace boundaries to avoid Type I singularities.
4. **Testing**: Verify kinematic calculations with a simulation tool (e.g., MATLAB) before hardware implementation.
5. **Link Length Adjustments**: If a different operating area is needed, adjust \( l_a \) and \( l_b \) while maintaining \( l_c = 2 l_a + k \) and \( l_b > l_a \).

## Safety Considerations

1. **Kinematic Limits**: Ensure the end-effector stays within the workspace to avoid singularities or mechanical limits.
2. **Interference**: The constraint \( l_c = 2 l_a + k \) prevents active link collisions. Verify \( k \geq 10 \, \text{mm} \) for safety.
3. **Validation**: Test inverse kinematics solutions to select the appropriate configuration (e.g., \( \theta_1 = \phi - \alpha \)).
4. **Monitoring**: Use real-time logging of joint angles and end-effector positions during operation.

## Future Extensions

The kinematic framework can be extended to:

- **Dynamic Analysis**: Incorporate velocity and acceleration (Section 3.2 of the paper) for control.
- **Singularity Handling**: Implement algorithms to detect and avoid singularity regions.
- **Optimization**: Optimize link lengths for specific tasks using workspace analysis tools.
- **3D Kinematics**: Extend to spatial 5-bar mechanisms for 3-DOF applications.

To implement new features, modify the kinematic computation functions and update workspace validation logic accordingly.
