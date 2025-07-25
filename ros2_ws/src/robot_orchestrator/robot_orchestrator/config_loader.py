#!/usr/bin/env python3
import yaml
import os
import logging
from ament_index_python.packages import get_package_share_directory

class ConfigLoader:
    """Configuration loader for robot parameters and assay definitions"""
    
    def __init__(self, config_file='assays.yaml'):
        self.config_file = config_file
        self.config = None
        self.robot_config = None
        self.assays_config = None
        
    def load_config(self):
        """Load configuration from YAML file with detailed logging"""
        try:
            package_share_dir = get_package_share_directory('robot_orchestrator')
            config_path = os.path.join(package_share_dir, 'config', self.config_file)
            
            print(f"🔧 Loading configuration from: {config_path}")
            
            if not os.path.exists(config_path):
                print(f"❌ Configuration file not found: {config_path}")
                return False
            
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            print(f"📄 Raw configuration loaded successfully")
            print(f"   Configuration keys: {list(self.config.keys())}")
            
            # Extract robot kinematics configuration
            self.robot_config = self._parse_robot_config()
            self._log_robot_config()
            
            # Extract assays configuration
            self.assays_config = self.config.get('assays', [])
            self._log_assays_config()
            
            # Validate complete configuration
            validation_results = self._validate_complete_config()
            self._log_validation_results(validation_results)
            
            return validation_results['config_valid']
            
        except Exception as e:
            print(f"❌ Error loading config: {e}")
            return False
    
    def _parse_robot_config(self):
        """Parse robot kinematics configuration from YAML"""
        kinematics_config = self.config.get('kinematics', [])
        
        if not kinematics_config:
            # Default configuration if not found in YAML
            return {
                'chain': '5_bar_parallel',
                'link_lengths': {
                    'l_a': 20.0,  # mm - proximal link length
                    'l_b': 40.0,  # mm - distal link length
                    'l_c': 50.0   # mm - base (locked) link length
                },
                'motor_config': {
                    'stall_torque': 0.32,    # kgcm
                    'stall_current': 0.2,    # A
                    'rated_torque': 19.6,    # mNm
                    'rated_current': 0.1     # A
                }
            }
        
        # Parse from YAML structure
        robot_data = kinematics_config[0]  # Assuming first entry is the robot config
        
        # Extract link lengths
        link_length_list = robot_data.get('link_length', [20.0, 40.0, 50.0])
        
        # Extract motor configuration
        motor_config = robot_data.get('motor config', {})
        
        return {
            'chain': robot_data.get('chain', '5_bar_parallel'),
            'link_lengths': {
                'l_a': link_length_list[0] if len(link_length_list) > 0 else 20.0,
                'l_b': link_length_list[1] if len(link_length_list) > 1 else 40.0,
                'l_c': link_length_list[2] if len(link_length_list) > 2 else 50.0
            },
            'motor_config': {
                'stall_torque': motor_config.get('stall_torque', 0.32),
                'stall_current': motor_config.get('stall_current', 0.2),
                'rated_torque': motor_config.get('rated_torque', 19.6),
                'rated_current': motor_config.get('rated_current', 0.1)
            }
        }
    
    def get_robot_config(self):
        """Get robot configuration"""
        return self.robot_config
    
    def get_assays_config(self):
        """Get assays configuration"""
        return self.assays_config
    
    def get_link_lengths(self):
        """Get link lengths for kinematics calculations"""
        if self.robot_config:
            lengths = self.robot_config['link_lengths']
            return lengths['l_a'], lengths['l_b'], lengths['l_c']
        return 20.0, 40.0, 50.0  # Default values
    
    def get_motor_config(self):
        """Get motor configuration"""
        if self.robot_config:
            return self.robot_config['motor_config']
        return {
            'stall_torque': 0.32,
            'stall_current': 0.2,
            'rated_torque': 19.6,
            'rated_current': 0.1
        }
    
    def get_experiment_config(self):
        """Get experiment configuration with start/end positions and timing"""
        if self.config:
            experiment_config = self.config.get('experiment', {})
            return {
                'start_position': experiment_config.get('start_position', [40.0, 25.0]),  # mm
                'end_position': experiment_config.get('end_position', [25.0, 25.0]),      # mm
                'reset_duration': experiment_config.get('reset_duration', 3.0),           # seconds
                'setup_duration': experiment_config.get('setup_duration', 2.0)            # seconds
            }
        return {
            'start_position': [40.0, 25.0],
            'end_position': [25.0, 25.0],
            'reset_duration': 3.0,
            'setup_duration': 2.0
        }
    
    def _log_robot_config(self):
        """Log detailed robot configuration information"""
        if not self.robot_config:
            print("⚠️  No robot configuration loaded")
            return
        
        print("\n🤖 Robot Configuration:")
        print(f"   Chain type: {self.robot_config['chain']}")
        
        lengths = self.robot_config['link_lengths']
        print(f"   Link lengths:")
        print(f"     l_a (proximal): {lengths['l_a']} mm")
        print(f"     l_b (distal):   {lengths['l_b']} mm")
        print(f"     l_c (base):     {lengths['l_c']} mm")
        
        # Calculate derived parameters
        r_min = abs(lengths['l_a'] - lengths['l_b'])
        r_max = lengths['l_a'] + lengths['l_b']
        print(f"   Workspace bounds:")
        print(f"     r_min: {r_min} mm")
        print(f"     r_max: {r_max} mm")
        
        motor = self.robot_config['motor_config']
        print(f"   Motor specifications:")
        print(f"     Stall torque:  {motor['stall_torque']} kgcm")
        print(f"     Stall current: {motor['stall_current']} A")
        print(f"     Rated torque:  {motor['rated_torque']} mNm")
        print(f"     Rated current: {motor['rated_current']} A")
    
    def _log_assays_config(self):
        """Log assays configuration information"""
        if not self.assays_config:
            print("⚠️  No assays configuration loaded")
            return
        
        print(f"\n🧪 Assays Configuration: {len(self.assays_config)} assays")
        
        for i, assay in enumerate(self.assays_config):
            name = assay.get('name', f'assay_{i+1}')
            n_trials = assay.get('n_trials', 0)
            duration = assay.get('trial_duration', 0)
            
            print(f"   Assay {i+1}: {name}")
            print(f"     Trials: {n_trials}, Duration: {duration}s")
            
            ff = assay.get('force_field', {})
            if ff.get('enabled', False):
                matrix = ff.get('matrix', [])
                if matrix:
                    ff_type = int(matrix[0]) if len(matrix) > 0 else 0
                    type_names = {0: 'static', 1: 'viscous_iso', 2: 'viscous_aniso', 
                                 3: 'viscous_oriented', 4: 'viscous_time'}
                    type_name = type_names.get(ff_type, 'unknown')
                    print(f"     Force field: {type_name} {matrix[1:]}")
                else:
                    print(f"     Force field: enabled (no parameters)")
            else:
                print(f"     Force field: disabled")
    
    def _validate_complete_config(self):
        """Validate the complete configuration and return detailed results"""
        results = {
            'config_valid': True,
            'robot_validation': {},
            'assays_validation': {},
            'warnings': [],
            'errors': []
        }
        
        # Validate robot configuration
        if self.robot_config:
            from .five_bar_kinematics import FiveBarKinematics
            
            lengths = self.robot_config['link_lengths']
            # Convert to meters for kinematics validation
            kinematics = FiveBarKinematics(lengths['l_a']/1000, lengths['l_b']/1000, lengths['l_c']/1000)
            results['robot_validation'] = kinematics.validation_results
            
            if not kinematics.validation_results['valid']:
                results['config_valid'] = False
                results['errors'].extend(kinematics.validation_results['errors'])
            
            results['warnings'].extend(kinematics.validation_results['warnings'])
        else:
            results['errors'].append("No robot configuration found")
            results['config_valid'] = False
        
        # Validate assays configuration
        if not self.assays_config:
            results['warnings'].append("No assays configured")
        else:
            assay_issues = self._validate_assays()
            results['assays_validation'] = assay_issues
            if assay_issues['errors']:
                results['config_valid'] = False
                results['errors'].extend(assay_issues['errors'])
            results['warnings'].extend(assay_issues['warnings'])
        
        return results
    
    def _validate_assays(self):
        """Validate assays configuration"""
        validation = {
            'valid_assays': 0,
            'total_trials': 0,
            'total_duration': 0.0,
            'errors': [],
            'warnings': []
        }
        
        for i, assay in enumerate(self.assays_config):
            assay_name = assay.get('name', f'assay_{i+1}')
            
            # Check required fields
            if 'n_trials' not in assay:
                validation['errors'].append(f"Assay '{assay_name}': missing 'n_trials'")
                continue
            
            if 'trial_duration' not in assay:
                validation['errors'].append(f"Assay '{assay_name}': missing 'trial_duration'")
                continue
            
            n_trials = assay.get('n_trials', 0)
            duration = assay.get('trial_duration', 0)
            
            # Validate values
            if n_trials <= 0:
                validation['warnings'].append(f"Assay '{assay_name}': n_trials <= 0")
            
            if duration <= 0:
                validation['warnings'].append(f"Assay '{assay_name}': trial_duration <= 0")
            
            # Check force field configuration
            ff = assay.get('force_field', {})
            if ff.get('enabled', False):
                matrix = ff.get('matrix', [])
                if not matrix:
                    validation['warnings'].append(f"Assay '{assay_name}': force field enabled but no matrix")
                elif len(matrix) < 2:
                    validation['warnings'].append(f"Assay '{assay_name}': insufficient force field parameters")
            
            validation['valid_assays'] += 1
            validation['total_trials'] += n_trials
            validation['total_duration'] += n_trials * duration
        
        return validation
    
    def _log_validation_results(self, results):
        """Log validation results with appropriate formatting"""
        print("\n🔍 Configuration Validation:")
        
        if results['config_valid']:
            print("   ✅ Configuration is valid")
        else:
            print("   ❌ Configuration has errors")
        
        # Log robot validation
        robot_val = results.get('robot_validation', {})
        if robot_val:
            print(f"   Robot workspace area: {robot_val.get('workspace_area', 0)*1e6:.1f} mm²")
            if robot_val.get('operating_area_fit', False):
                print("   ✅ 40mm×30mm operating area fits in workspace")
            else:
                print("   ⚠️  Operating area may not fit optimally")
        
        # Log assays validation
        assays_val = results.get('assays_validation', {})
        if assays_val:
            print(f"   Valid assays: {assays_val.get('valid_assays', 0)}")
            print(f"   Total trials: {assays_val.get('total_trials', 0)}")
            print(f"   Estimated duration: {assays_val.get('total_duration', 0):.1f}s")
        
        # Log warnings
        if results['warnings']:
            print(f"\n⚠️  Warnings ({len(results['warnings'])}):")
            for warning in results['warnings']:
                print(f"     • {warning}")
        
        # Log errors
        if results['errors']:
            print(f"\n❌ Errors ({len(results['errors'])}):")
            for error in results['errors']:
                print(f"     • {error}")
        
        print()  # Empty line for readability
