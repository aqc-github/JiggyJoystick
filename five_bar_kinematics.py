import numpy as np
import warnings

class FiveBarKinematics:
    def __init__(self, l_a, l_b, l_c):
        self.l_a = l_a
        self.l_b = l_b
        self.l_c = l_c
        
        # Workspace limits
        self.workspace_limits = self._calculate_workspace_limits()
        
        # Validate configuration on initialization
        self.validation_results = self._validate_configuration()

    def direct_kinematics(self, theta1, theta2):
        """Compute the end-effector position (x_P, y_P) from joint angles."""
        try:
            E = 2 * self.l_a * (1 - (np.cos(theta1) - np.cos(theta2)) ** 2)
            F = 2 * self.l_a * (np.sin(theta1) - np.sin(theta2))
            G = self.l_a**2 + 2 * self.l_b**2 + 2 * self.l_a * self.l_b * (np.cos(theta1) - np.cos(theta2)) - self.l_c**2

            # Check for singularity conditions
            discriminant = F**2 + G**2
            if discriminant < 0:
                warnings.warn(f"Direct kinematics: negative discriminant {discriminant}")
                return np.array([np.nan, np.nan])
            
            if np.abs(G - E) < 1e-10:
                warnings.warn(f"Direct kinematics: singular configuration (G-E ≈ 0)")
                return np.array([np.nan, np.nan])

            theta3 = 2 * np.arctan((-F + np.sqrt(discriminant)) / (G - E))

            x_P = self.l_a * np.cos(theta1) + self.l_b * np.cos(theta3)
            y_P = self.l_a * np.sin(theta1) + self.l_b * np.sin(theta3)

            return np.array([x_P, y_P])
            
        except Exception as e:
            warnings.warn(f"Direct kinematics error: {e}")
            return np.array([np.nan, np.nan])

    def inverse_kinematics(self, x, y):
        """Compute joint angles (theta1, theta2) from the end-effector position (x, y)."""
        try:
            # Check if position is within workspace
            if not self.is_position_reachable(x, y):
                warnings.warn(f"Position ({x:.3f}, {y:.3f}) is outside workspace")
                return np.array([np.nan, np.nan])
            
            d_OP = np.sqrt(x**2 + y**2)
            d_BP = np.sqrt((x - self.l_c)**2 + y**2)
            
            # Check for degenerate cases
            if d_OP < 1e-10 or d_BP < 1e-10:
                warnings.warn(f"Degenerate case: point too close to base joints")
                return np.array([np.nan, np.nan])

            # Left chain (O-A-P)
            cos_alpha = (self.l_a**2 + d_OP**2 - self.l_b**2) / (2 * self.l_a * d_OP)
            
            # Check if solution exists for left chain
            if abs(cos_alpha) > 1.0:
                warnings.warn(f"Left chain: no solution (cos_alpha = {cos_alpha:.3f})")
                return np.array([np.nan, np.nan])
            
            phi = np.arctan2(y, x)
            alpha = np.arccos(np.clip(cos_alpha, -1.0, 1.0))
            theta1 = phi - alpha

            # Right chain (B-C-P)
            cos_beta = (self.l_a**2 + d_BP**2 - self.l_b**2) / (2 * self.l_a * d_BP)
            
            # Check if solution exists for right chain
            if abs(cos_beta) > 1.0:
                warnings.warn(f"Right chain: no solution (cos_beta = {cos_beta:.3f})")
                return np.array([np.nan, np.nan])
            
            psi = np.arctan2(y, x - self.l_c)
            beta = np.arccos(np.clip(cos_beta, -1.0, 1.0))
            theta2 = psi + beta

            return np.array([theta1, theta2])
            
        except Exception as e:
            warnings.warn(f"Inverse kinematics error: {e}")
            return np.array([np.nan, np.nan])
    
    def _calculate_workspace_limits(self):
        """Calculate workspace boundaries for the 5-bar parallel robot."""
        # Workspace is intersection of two annular regions
        r_min = abs(self.l_a - self.l_b)
        r_max = self.l_a + self.l_b
        
        return {
            'r_min': r_min,
            'r_max': r_max,
            'center_O': np.array([0.0, 0.0]),
            'center_B': np.array([self.l_c, 0.0])
        }
    
    def is_position_reachable(self, x, y):
        """Check if a position (x, y) is within the robot workspace."""
        limits = self.workspace_limits
        
        # Distance from O
        d_O = np.sqrt(x**2 + y**2)
        # Distance from B
        d_B = np.sqrt((x - self.l_c)**2 + y**2)
        
        # Position must be reachable from both base joints
        reachable_from_O = limits['r_min'] <= d_O <= limits['r_max']
        reachable_from_B = limits['r_min'] <= d_B <= limits['r_max']
        
        return reachable_from_O and reachable_from_B
    
    def _validate_configuration(self):
        """Validate robot configuration and identify potential issues."""
        results = {
            'valid': True,
            'warnings': [],
            'errors': [],
            'workspace_area': 0.0,
            'operating_area_fit': False
        }
        
        # Check link length ratios
        if self.l_a <= 0 or self.l_b <= 0 or self.l_c <= 0:
            results['errors'].append("All link lengths must be positive")
            results['valid'] = False
        
        # Check interference constraint: l_c should be >= 2*l_a for safety
        if self.l_c < 2 * self.l_a:
            results['warnings'].append(f"Base length ({self.l_c:.1f}) may cause interference. Recommended: >= {2*self.l_a:.1f}")
        
        # Check for degenerate configurations
        if self.l_a == self.l_b:
            results['warnings'].append("Equal link lengths may cause singular configurations")
        
        if abs(self.l_a - self.l_b) > 0.8 * min(self.l_a, self.l_b):
            results['warnings'].append("Large link length ratio may reduce workspace quality")
        
        # Calculate approximate workspace area (simplified)
        if results['valid']:
            r_min = abs(self.l_a - self.l_b)
            r_max = self.l_a + self.l_b
            
            # Approximate workspace as intersection of two circles
            # This is a simplification - actual calculation is more complex
            circle_area = np.pi * (r_max**2 - r_min**2)
            results['workspace_area'] = circle_area * 0.6  # Rough approximation
        
        # Check if 40mm x 30mm operating area fits
        target_area = 0.040 * 0.030  # Convert to m²
        if results['workspace_area'] > target_area * 2:  # Factor of 2 for safety margin
            results['operating_area_fit'] = True
        else:
            results['warnings'].append("Workspace may be too small for 40mm x 30mm operating area")
        
        return results
    
    def get_workspace_boundary_points(self, n_points=100):
        """Generate points on the workspace boundary for visualization."""
        theta = np.linspace(0, 2*np.pi, n_points)
        
        # Generate boundary points (simplified - just outer boundary)
        r_max = self.l_a + self.l_b
        
        # Points reachable from O
        boundary_O_outer = np.array([[r_max * np.cos(t), r_max * np.sin(t)] for t in theta])
        
        # Points reachable from B
        boundary_B_outer = np.array([[self.l_c + r_max * np.cos(t), r_max * np.sin(t)] for t in theta])
        
        return boundary_O_outer, boundary_B_outer
    
    def check_singularity_at_position(self, x, y):
        """Check if a position is near a singularity."""
        # Type I singularities occur at workspace boundaries
        d_O = np.sqrt(x**2 + y**2)
        d_B = np.sqrt((x - self.l_c)**2 + y**2)
        
        r_min = abs(self.l_a - self.l_b)
        r_max = self.l_a + self.l_b
        
        # Check proximity to boundaries (within 5% of limits)
        tolerance = 0.05 * (r_max - r_min)
        
        singularity_types = []
        
        if abs(d_O - r_min) < tolerance or abs(d_O - r_max) < tolerance:
            singularity_types.append("Type I (left chain boundary)")
        
        if abs(d_B - r_min) < tolerance or abs(d_B - r_max) < tolerance:
            singularity_types.append("Type I (right chain boundary)")
        
        return singularity_types
