#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import time
import csv
import os
from datetime import datetime
from custom_interfaces.action import TrialAction
from custom_interfaces.srv import LoadConfig
from .five_bar_kinematics import FiveBarKinematics
from .config_loader import ConfigLoader


class SimpleTestNode(Node):
    """Simple node to test basic communication with Teensy"""
    
    def __init__(self):
        super().__init__('simple_test_node')
        
        # Publishers
        self.handshake_pub = self.create_publisher(Bool, '/ros2_handshake', 10)
        self.start_trial_pub = self.create_publisher(Bool, '/start_trial', 10)
        self.abort_trial_pub = self.create_publisher(Bool, '/abort_trial', 10)
        self.trial_success_pub = self.create_publisher(Bool, '/trial_success', 10)
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        self.teensy_handshake_sub = self.create_subscription(
            Bool, '/microcontroller_handshake', self.teensy_handshake_callback, 10
        )
        
        # State
        self.handshake_complete = False
        self.joint_states_received = False
        self.joint_message_count = 0
        self.last_joint_time = time.time()
        
        # Timer for status updates and test commands
        self.status_timer = self.create_timer(2.0, self.print_status)
        self.test_timer = self.create_timer(10.0, self.send_test_command)
        self.test_command_count = 0
        
        self.get_logger().info('🚀 Simple Test Node initialized')
        self.get_logger().info('Waiting for handshake from Teensy...')
    
    def joint_states_callback(self, msg):
        """Handle joint state messages from Teensy"""
        self.joint_states_received = True
        self.joint_message_count += 1
        self.last_joint_time = time.time()
        
        # Log first few messages
        if self.joint_message_count <= 5:
            self.get_logger().info(f'📊 Joint states: pos={msg.position[:2]}, vel={msg.velocity[:2]}')
    
    def teensy_handshake_callback(self, msg):
        """Handle handshake from Teensy"""
        if msg.data and not self.handshake_complete:
            self.get_logger().info('🤝 Received handshake from Teensy!')
            
            # Send handshake response
            response = Bool()
            response.data = True
            self.handshake_pub.publish(response)
            
            self.handshake_complete = True
            self.get_logger().info('✅ Handshake complete! Communication established.')
    
    def print_status(self):
        """Print current communication status"""
        status = f"📊 Communication Status:\n"
        status += f"  Handshake: {'✅' if self.handshake_complete else '❌'} {'Complete' if self.handshake_complete else 'Waiting'}\n"
        status += f"  Joint States: {'✅' if self.joint_states_received else '❌'} {'Receiving' if self.joint_states_received else 'No data'}\n"
        
        if self.joint_states_received:
            time_since_last = time.time() - self.last_joint_time
            status += f"  Messages received: {self.joint_message_count}\n"
            status += f"  Last message: {time_since_last:.1f}s ago\n"
            
            if time_since_last > 5.0:
                status += "  ⚠️  Warning: No recent joint states!\n"
        
        self.get_logger().info(status)
    
    def send_test_command(self):
        """Send test commands to Teensy"""
        if not self.handshake_complete:
            return
            
        self.test_command_count += 1
        
        if self.test_command_count % 3 == 1:
            self.get_logger().info('📤 Sending start trial command...')
            msg = Bool()
            msg.data = True
            self.start_trial_pub.publish(msg)
            
        elif self.test_command_count % 3 == 2:
            self.get_logger().info('📤 Sending trial success command...')
            msg = Bool()
            msg.data = True
            self.trial_success_pub.publish(msg)
            
        else:
            self.get_logger().info('📤 Sending abort trial command...')
            msg = Bool()
            msg.data = True
            self.abort_trial_pub.publish(msg)


def simple_test_main(args=None):
    rclpy.init(args=args)
    node = SimpleTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


class ExperimentManagerNode(Node):
    """Node that orchestrates experiments based on assays.yaml configuration"""
    
    def __init__(self):
        super().__init__('experiment_manager_node')
        
        # Parameters
        self.declare_parameter('config_file', 'assays.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        # Configuration
        self.config_loader = ConfigLoader(config_file)
        self.config = None
        self.assays = []
        self.current_assay = 0
        self.current_trial = 0
        
        # Load configuration
        self.load_config(config_file)
        
        # Action client for trial execution
        self.trial_action_client = ActionClient(self, TrialAction, 'trial_action')
        
        # Service for dynamic config reloading
        self.load_config_service = self.create_service(
            LoadConfig, 'load_config', self.load_config_callback
        )
        
        self.get_logger().info('📋 Experiment Manager Node initialized')
        self.get_logger().info(f'Loaded {len(self.assays)} assays')
    
    def load_config(self, config_file):
        """Load experiment configuration from YAML file"""
        try:
            self.config_loader = ConfigLoader(config_file)
            if self.config_loader.load_config():
                self.config = self.config_loader.config
                self.assays = self.config_loader.get_assays_config()
                robot_config = self.config_loader.get_robot_config()
                
                self.get_logger().info(f'✅ Loaded {len(self.assays)} assays from {config_file}')
                self.get_logger().info(f'🤖 Robot config: {robot_config["chain"]} with links {self.config_loader.get_link_lengths()}')
            else:
                self.get_logger().error(f'❌ Failed to load config from {config_file}')
                self.assays = []
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load config: {e}')
            self.assays = []
    
    def load_config_callback(self, request, response):
        """Service callback for dynamic config reloading"""
        try:
            self.load_config(request.config_file_path)
            response.success = True
            response.message = f"Configuration loaded successfully: {len(self.assays)} assays"
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response
    
    def run_experiment(self):
        """Run the complete experiment"""
        self.get_logger().info('🚀 Starting experiment...')
        
        if not self.assays:
            self.get_logger().error('❌ No assays configured! Please check assays.yaml')
            return
        
        # Log experiment overview
        total_trials = sum(assay.get('n_trials', 0) for assay in self.assays)
        self.get_logger().info(f'Experiment overview: {len(self.assays)} assays, {total_trials} total trials')
        
        for i, assay in enumerate(self.assays):
            n_trials = assay.get('n_trials', 0)
            duration = assay.get('trial_duration', 0)
            ff_enabled = assay.get('force_field', {}).get('enabled', False)
            self.get_logger().info(f'  Assay {i+1}: {n_trials} trials, {duration}s each, force field: {ff_enabled}')
        
        # Wait for action server to be available
        self.get_logger().info('⏳ Waiting for action server...')
        if not self.trial_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Action server not available!')
            return
        
        self.get_logger().info('✅ Action server ready')
        
        # Execute each assay
        for assay_idx, assay in enumerate(self.assays):
            self.current_assay = assay_idx
            self.get_logger().info(f'\n📋 Starting Assay {assay_idx + 1}/{len(self.assays)}')
            
            n_trials = assay.get('n_trials', 0)
            trial_duration = assay.get('trial_duration', 5.0)
            force_field = assay.get('force_field', {})
            
            # Execute each trial in the assay
            for trial_idx in range(n_trials):
                self.current_trial = trial_idx
                self.get_logger().info(f'\n🧪 Starting Trial {trial_idx + 1}/{n_trials}')
                
                # Create trial goal
                goal = TrialAction.Goal()
                goal.assay_number = assay_idx + 1
                goal.trial_number = trial_idx + 1
                goal.duration = trial_duration
                goal.force_field_enabled = force_field.get('enabled', False)
                goal.force_field_matrix = force_field.get('matrix', [0.0, 0.0, 0.0, 0.0])
                
                # Send goal to action server
                future = self.trial_action_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, future)
                
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().error(f'❌ Trial {trial_idx + 1} rejected!')
                    continue
                
                self.get_logger().info(f'✅ Trial {trial_idx + 1} accepted, executing...')
                
                # Wait for trial to complete
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                
                result = result_future.result()
                if result.result.success:
                    self.get_logger().info(f'✅ Trial {trial_idx + 1} completed successfully')
                else:
                    self.get_logger().warn(f'⚠️ Trial {trial_idx + 1} failed')
                
                # Brief pause between trials
                time.sleep(1.0)
            
            self.get_logger().info(f'✅ Assay {assay_idx + 1} completed')
        
        self.get_logger().info('\n🎆 Experiment completed successfully!')
        
        # Run post-experiment analysis
        self.run_post_experiment_analysis()
    
    def run_post_experiment_analysis(self):
        """Run post-experiment data analysis and visualization"""
        self.get_logger().info('\n📊 Starting post-experiment analysis...')
        
        try:
            import subprocess
            import sys
            
            # Run the experiment analyzer script
            result = subprocess.run([
                sys.executable, '/ros2_ws/experiment_analyzer.py', '--no-show'
            ], capture_output=True, text=True, timeout=60)
            
            if result.returncode == 0:
                self.get_logger().info('✅ Post-experiment analysis completed successfully!')
                self.get_logger().info('📈 Analysis plots saved to analysis_output/ directory')
                
                # Log some of the analysis output
                if result.stdout:
                    lines = result.stdout.split('\n')
                    for line in lines[-20:]:  # Show last 20 lines
                        if line.strip():
                            self.get_logger().info(f'Analysis: {line}')
            else:
                self.get_logger().error(f'❌ Post-experiment analysis failed with code {result.returncode}')
                if result.stderr:
                    self.get_logger().error(f'Error: {result.stderr}')
                    
        except subprocess.TimeoutExpired:
            self.get_logger().error('⏰ Post-experiment analysis timed out after 60 seconds')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to run post-experiment analysis: {e}')


class ControlNode(Node):
    """Node that manages trial execution and robot control"""
    
    def __init__(self):
        super().__init__('control_node')
        
        # Load robot configuration
        self.config_loader = ConfigLoader()
        if not self.config_loader.load_config():
            self.get_logger().error('❌ Failed to load robot configuration!')
        
        # Get robot parameters from configuration
        l_a, l_b, l_c = self.config_loader.get_link_lengths()
        self.motor_config = self.config_loader.get_motor_config()
        
        # Initialize 5-bar parallel robot kinematics
        # Convert from mm to meters for calculations
        self.kinematics = FiveBarKinematics(l_a / 1000.0, l_b / 1000.0, l_c / 1000.0)
        
        self.get_logger().info(f'🤖 Robot Configuration Loaded:')
        self.get_logger().info(f'  Link lengths: l_a={l_a}mm, l_b={l_b}mm, l_c={l_c}mm')
        self.get_logger().info(f'  Motor config: {self.motor_config}')
        
        # Action server for trial execution
        self.trial_action_server = ActionServer(
            self, TrialAction, 'trial_action', 
            self.execute_trial_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Publishers
        self.handshake_pub = self.create_publisher(Bool, '/ros2_handshake', 10)
        self.start_trial_pub = self.create_publisher(Bool, '/start_trial', 10)
        self.abort_trial_pub = self.create_publisher(Bool, '/abort_trial', 10)
        self.trial_success_pub = self.create_publisher(Bool, '/trial_success', 10)
        
        # Additional publishers for trial metadata
        self.assay_pub = self.create_publisher(Int32, '/assay_number', 10)
        self.trial_pub = self.create_publisher(Int32, '/trial_number', 10)
        self.trial_status_pub = self.create_publisher(Int32, '/trial_status', 10)
        
        # Force field and torque publishers
        self.torque_pub = self.create_publisher(Float64MultiArray, '/torque_commands', 10)
        self.ff_enabled_pub = self.create_publisher(Bool, '/ff_enabled', 10)
        self.ff_value_pub = self.create_publisher(Float64MultiArray, '/ff_value', 10)
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        self.teensy_handshake_sub = self.create_subscription(
            Bool, '/microcontroller_handshake', self.teensy_handshake_callback, 10
        )
        
        # State
        self.handshake_complete = False
        self.q = None
        self.dq = None
        self.current_goal = None
        self.trial_start_time = None
        
        # Control parameters
        self.force_field_enabled = False
        self.force_field_matrix = np.array([[0.0, 0.0], [0.0, 0.0]])
        
        # Dynamic force field parameters
        self.ff_type = 'static'  # 'static', 'viscous_iso', 'viscous_aniso', 'viscous_oriented', 'viscous_time'
        self.ff_params = {}
        self.ff_start_time = None
        
        # Robot parameters are now loaded from configuration
        # Parameters stored in self.kinematics and self.motor_config
        
        # Control loop timer (100 Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('🎮 Control Node initialized')
        self.get_logger().info('Action server "/trial_action" ready')
    
    def forward_kinematics(self, theta1, theta2):
        """Calculate forward kinematics for 5-bar parallel robot"""
        return self.kinematics.direct_kinematics(theta1, theta2)
    
    def inverse_kinematics(self, x, y):
        """Calculate inverse kinematics for 5-bar parallel robot"""
        return self.kinematics.inverse_kinematics(x, y)
    
    def jacobian(self, theta1, theta2):
        """Calculate Jacobian matrix for 5-bar parallel robot"""
        # Numerical differentiation for Jacobian calculation
        delta = 1e-6
        
        # Current position
        p0 = self.forward_kinematics(theta1, theta2)
        
        # Partial derivatives w.r.t. theta1
        p1 = self.forward_kinematics(theta1 + delta, theta2)
        dxdth1 = (p1[0] - p0[0]) / delta
        dydth1 = (p1[1] - p0[1]) / delta
        
        # Partial derivatives w.r.t. theta2
        p2 = self.forward_kinematics(theta1, theta2 + delta)
        dxdth2 = (p2[0] - p0[0]) / delta
        dydth2 = (p2[1] - p0[1]) / delta
        
        J = np.array([[dxdth1, dxdth2],
                      [dydth1, dydth2]])
        
        return J
    
    def calculate_dynamic_force_field(self, ee_pos, ee_vel):
        """Calculate dynamic force field forces based on type and parameters"""
        if self.ff_type == 'static':
            # Original static force field
            return self.force_field_matrix @ ee_vel
            
        elif self.ff_type == 'viscous_iso':
            # Isotropic viscous field: F = -b * v
            b = self.ff_params.get('damping', 0.0)
            return -b * ee_vel
            
        elif self.ff_type == 'viscous_aniso':
            # Anisotropic viscous field: F = -diag(bx, by) * v
            bx = self.ff_params.get('damping_x', 0.0)
            by = self.ff_params.get('damping_y', 0.0)
            B = np.array([[bx, 0.0], [0.0, by]])
            return -B @ ee_vel
            
        elif self.ff_type == 'viscous_oriented':
            # Oriented viscous field: rotated damping matrix
            b_major = self.ff_params.get('damping_major', 0.0)
            b_minor = self.ff_params.get('damping_minor', 0.0)
            angle = self.ff_params.get('angle', 0.0)  # in radians
            
            # Create damping matrix in principal axes
            B_principal = np.array([[b_major, 0.0], [0.0, b_minor]])
            
            # Rotation matrix
            c, s = np.cos(angle), np.sin(angle)
            R = np.array([[c, -s], [s, c]])
            
            # Rotate damping matrix: B = R * B_principal * R^T
            B = R @ B_principal @ R.T
            return -B @ ee_vel
            
        elif self.ff_type == 'viscous_time':
            # Time-dependent viscous field
            if self.ff_start_time is None:
                return np.zeros(2)
                
            elapsed = (self.get_clock().now() - self.ff_start_time).nanoseconds / 1e9
            
            # Get time-dependent parameters
            b_initial = self.ff_params.get('damping_initial', 0.0)
            b_final = self.ff_params.get('damping_final', 0.0)
            transition_time = self.ff_params.get('transition_time', 1.0)
            
            # Linear interpolation
            if elapsed <= transition_time:
                t_norm = elapsed / transition_time
                b_current = b_initial + (b_final - b_initial) * t_norm
            else:
                b_current = b_final
                
            return -b_current * ee_vel
            
        else:
            # Unknown type, return zero forces
            self.get_logger().warn(f'Unknown force field type: {self.ff_type}')
            return np.zeros(2)
    
    def control_loop(self):
        """Main control loop running at 100Hz"""
        if self.q is None or self.dq is None:
            return
            
        # Calculate forward kinematics
        ee_pos = self.forward_kinematics(self.q[0], self.q[1])
        
        # Calculate Jacobian
        J = self.jacobian(self.q[0], self.q[1])
        
        # Calculate end-effector velocity
        ee_vel = J @ self.dq[:2]
        
        # Calculate force field forces
        ff_forces = np.zeros(2)
        if self.force_field_enabled:
            ff_forces = self.calculate_dynamic_force_field(ee_pos, ee_vel)
        
        # Convert forces to joint torques using Jacobian transpose
        try:
            torques = J.T @ ff_forces
        except:
            torques = np.zeros(2)
        
        # Publish torque commands
        torque_msg = Float64MultiArray()
        torque_msg.data = torques.tolist()
        self.torque_pub.publish(torque_msg)
        
        # Update feedback with torques if we have an active trial
        if self.current_goal and hasattr(self.current_goal, 'request'):
            feedback = TrialAction.Feedback()
            feedback.joint_positions = self.q.tolist()
            feedback.joint_velocities = self.dq.tolist()
            feedback.torques = torques.tolist()
            # Note: We'll publish this in the trial execution loop
    
    def joint_states_callback(self, msg):
        """Handle joint state messages from Teensy"""
        self.q = np.array(msg.position)
        self.dq = np.array(msg.velocity)
    
    def teensy_handshake_callback(self, msg):
        """Handle handshake from Teensy"""
        if msg.data and not self.handshake_complete:
            self.get_logger().info('🤝 Received handshake from Teensy!')
            
            # Send handshake response
            response = Bool()
            response.data = True
            self.handshake_pub.publish(response)
            
            self.handshake_complete = True
            self.get_logger().info('✅ Handshake complete! Communication established.')
    
    def goal_callback(self, goal_request):
        """Accept or reject incoming action goals"""
        if not self.handshake_complete:
            self.get_logger().warn('❌ Goal rejected: Handshake not complete')
            return GoalResponse.REJECT
        
        self.get_logger().info(f'✅ Goal accepted: Assay {goal_request.assay_number}, Trial {goal_request.trial_number}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests"""
        self.get_logger().info('🛑 Trial cancellation requested')
        return CancelResponse.ACCEPT
    
    def execute_trial_callback(self, goal_handle):
        """Execute a trial based on the action goal"""
        goal = goal_handle.request
        result = TrialAction.Result()
        
        self.get_logger().info(f'🚀 Starting trial: Assay {goal.assay_number}, Trial {goal.trial_number}')
        
        # Publish trial metadata
        self.assay_pub.publish(Int32(data=goal.assay_number))
        self.trial_pub.publish(Int32(data=goal.trial_number))
        self.trial_status_pub.publish(Int32(data=1))  # Status: Setup successful
        
        # Signal trial start to Teensy
        self.start_trial_pub.publish(Bool(data=True))
        
        # Configure force field for trial
        self.force_field_enabled = goal.force_field_enabled
        if self.force_field_enabled:
            # Parse force field configuration
            self.parse_force_field_config(goal.force_field_matrix)
            self.ff_start_time = self.get_clock().now()
            self.get_logger().info(f'✨ Force field enabled: type={self.ff_type}, params={self.ff_params}')
        else:
            self.force_field_matrix = np.array([[0.0, 0.0], [0.0, 0.0]])
            self.ff_type = 'static'
            self.ff_params = {}
            self.ff_start_time = None
            self.get_logger().info('⚫ Force field disabled')
        
        # Publish force field configuration
        self.ff_enabled_pub.publish(Bool(data=self.force_field_enabled))
        self.ff_value_pub.publish(Float64MultiArray(data=goal.force_field_matrix))
        
        # Store trial information
        self.current_goal = goal_handle
        self.trial_start_time = self.get_clock().now()
        
        # Execute trial for the specified duration
        trial_duration = goal.duration
        feedback = TrialAction.Feedback()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('🛑 Trial cancelled')
                self.abort_trial_pub.publish(Bool(data=True))
                self.current_goal = None
                goal_handle.canceled()
                result.success = False
                return result
            
            # Calculate elapsed time
            elapsed = (self.get_clock().now() - self.trial_start_time).nanoseconds / 1e9
            
            # Publish feedback
            if self.q is not None and self.dq is not None:
                # Calculate torques for feedback
                try:
                    ee_pos = self.forward_kinematics(self.q[0], self.q[1])
                    J = self.jacobian(self.q[0], self.q[1])
                    ee_vel = J @ self.dq[:2]
                    ff_forces = np.zeros(2)
                    if self.force_field_enabled:
                        ff_forces = self.calculate_dynamic_force_field(ee_pos, ee_vel)
                    torques = J.T @ ff_forces
                except:
                    torques = np.zeros(2)
                
                feedback.joint_positions = self.q.tolist()
                feedback.joint_velocities = self.dq.tolist()
                feedback.torques = torques.tolist()
                goal_handle.publish_feedback(feedback)
            
            # Check if trial is complete
            if elapsed >= trial_duration:
                self.get_logger().info(f'✅ Trial completed successfully after {elapsed:.1f}s')
                self.trial_success_pub.publish(Bool(data=True))
                result.success = True
                self.current_goal = None
                goal_handle.succeed()
                return result
            
            # Sleep briefly
            time.sleep(0.01)
        
        # If we get here, something went wrong
        self.get_logger().error('❌ Trial execution interrupted')
        self.abort_trial_pub.publish(Bool(data=True))
        result.success = False
        self.current_goal = None
        goal_handle.abort()
        return result
    
    def print_status(self):
        """Print current status"""
        status = f"🎮 Control Status:\n"
        status += f"  Handshake: {'✅' if self.handshake_complete else '❌'} {'Complete' if self.handshake_complete else 'Waiting'}\n"
        status += f"  Joint States: {'✅' if self.q is not None else '❌'} {'Receiving' if self.q is not None else 'No data'}\n"
        status += f"  Action Server: {'✅' if self.trial_action_server else '❌'} {'Ready' if self.trial_action_server else 'Not ready'}\n"
        
        if self.q is not None:
            status += f"  Current positions: [{self.q[0]:.3f}, {self.q[1]:.3f}]\n"
            if self.dq is not None:
                status += f"  Current velocities: [{self.dq[0]:.3f}, {self.dq[1]:.3f}]\n"
            
        status += f"  Force field: {'✅' if self.force_field_enabled else '❌'} {'Enabled' if self.force_field_enabled else 'Disabled'}\n"
        
        if self.current_goal:
            status += f"  Current trial: Assay {self.current_goal.request.assay_number}, Trial {self.current_goal.request.trial_number}\n"
            
        self.get_logger().info(status)
    
    def parse_force_field_config(self, ff_matrix):
        """Parse force field configuration from matrix format"""
        # Convert matrix to configuration
        # Format: [type_id, param1, param2, param3, ...]
        # type_id: 0=static, 1=viscous_iso, 2=viscous_aniso, 3=viscous_oriented, 4=viscous_time
        
        if len(ff_matrix) < 1:
            self.ff_type = 'static'
            self.force_field_matrix = np.array([[0.0, 0.0], [0.0, 0.0]])
            self.ff_params = {}
            return
            
        type_id = int(ff_matrix[0])
        
        if type_id == 0:  # Static (original behavior)
            self.ff_type = 'static'
            if len(ff_matrix) >= 5:
                self.force_field_matrix = np.array([[ff_matrix[1], ff_matrix[2]], [ff_matrix[3], ff_matrix[4]]])
            else:
                self.force_field_matrix = np.array([[0.0, 0.0], [0.0, 0.0]])
            self.ff_params = {}
            
        elif type_id == 1:  # Viscous isotropic
            self.ff_type = 'viscous_iso'
            self.ff_params = {
                'damping': ff_matrix[1] if len(ff_matrix) > 1 else 0.0
            }
            
        elif type_id == 2:  # Viscous anisotropic
            self.ff_type = 'viscous_aniso'
            self.ff_params = {
                'damping_x': ff_matrix[1] if len(ff_matrix) > 1 else 0.0,
                'damping_y': ff_matrix[2] if len(ff_matrix) > 2 else 0.0
            }
            
        elif type_id == 3:  # Viscous oriented
            self.ff_type = 'viscous_oriented'
            self.ff_params = {
                'damping_major': ff_matrix[1] if len(ff_matrix) > 1 else 0.0,
                'damping_minor': ff_matrix[2] if len(ff_matrix) > 2 else 0.0,
                'angle': ff_matrix[3] if len(ff_matrix) > 3 else 0.0  # in radians
            }
            
        elif type_id == 4:  # Viscous time-dependent
            self.ff_type = 'viscous_time'
            self.ff_params = {
                'damping_initial': ff_matrix[1] if len(ff_matrix) > 1 else 0.0,
                'damping_final': ff_matrix[2] if len(ff_matrix) > 2 else 0.0,
                'transition_time': ff_matrix[3] if len(ff_matrix) > 3 else 1.0
            }
            
        else:
            # Unknown type, default to static
            self.get_logger().warn(f'Unknown force field type ID: {type_id}, defaulting to static')
            self.ff_type = 'static'
            self.force_field_matrix = np.array([[0.0, 0.0], [0.0, 0.0]])
            self.ff_params = {}


class LoggerNode(Node):
    """Node that logs experiment data to CSV files"""
    
    def __init__(self):
        super().__init__('logger_node')
        
        # Create logs directory
        self.log_dir = os.path.join(os.getcwd(), 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # CSV writing state
        self.csv_file = None
        self.csv_writer = None
        
        # Data storage
        self.data = {}
        self.current_assay = None
        self.current_trial = None
        self.trial_active = False
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        
        # Subscribe to trial metadata
        self.assay_sub = self.create_subscription(
            Int32, '/assay_number', self.assay_callback, 10
        )
        self.trial_sub = self.create_subscription(
            Int32, '/trial_number', self.trial_callback, 10
        )
        self.trial_start_sub = self.create_subscription(
            Bool, '/start_trial', self.trial_start_callback, 10
        )
        self.trial_success_sub = self.create_subscription(
            Bool, '/trial_success', self.trial_success_callback, 10
        )
        
        self.get_logger().info('📊 Logger Node initialized')
        self.get_logger().info(f'Logs will be saved to: {self.log_dir}')
    
    def assay_callback(self, msg):
        """Handle assay number updates"""
        self.current_assay = msg.data
        
    def trial_callback(self, msg):
        """Handle trial number updates"""
        self.current_trial = msg.data
        
    def trial_start_callback(self, msg):
        """Handle trial start signals"""
        if msg.data and self.current_assay is not None:
            # Close any existing CSV file
            if self.csv_file:
                self.close_csv_writer()
            
            # Create new CSV file for this trial
            self.create_csv_writer(self.current_assay)
            self.trial_active = True
            
    def trial_success_callback(self, msg):
        """Handle trial completion signals"""
        if msg.data:
            self.trial_active = False
            self.close_csv_writer()
    
    def joint_states_callback(self, msg):
        """Handle joint state messages for logging"""
        self.data['joint_positions'] = msg.position
        self.data['joint_velocities'] = msg.velocity
        self.data['joint_efforts'] = msg.effort
        
        # Log data to CSV if writer is active and trial is running
        if self.csv_writer and self.trial_active:
            self.log_data()
        
        # For now, just log occasionally
        if hasattr(self, 'log_count'):
            self.log_count += 1
        else:
            self.log_count = 1
            
        if self.log_count % 500 == 1:  # Log every 500 messages (~10 seconds)
            self.get_logger().info(f'📊 Logged {self.log_count} joint state messages')
    
    def create_csv_writer(self, assay_number):
        """Create a CSV writer for logging trial data"""
        current_time = datetime.now().strftime('%H-%M-%S-%d-%m')
        filename = f'{current_time}-assay{assay_number}.csv'
        file_path = os.path.join(self.log_dir, filename)

        self.csv_file = open(file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write headers
        headers = ['timestamp', 'joint_positions', 'joint_velocities', 'joint_efforts']
        self.csv_writer.writerow(headers)
        
        self.get_logger().info(f'Created CSV log file: {filename}')

    def log_data(self):
        """Log current data to CSV"""
        if not self.csv_writer:
            return

        # Get current time
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Include milliseconds

        row = [
            current_time,
            list(self.data.get('joint_positions', [])),
            list(self.data.get('joint_velocities', [])),
            list(self.data.get('joint_efforts', []))
        ]
        self.csv_writer.writerow(row)
        
        # Flush to ensure data is written immediately
        self.csv_file.flush()

    def close_csv_writer(self):
        """Close the CSV file"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.get_logger().info('CSV log file closed')


# Entry points for each node
def experiment_manager_main(args=None):
    rclpy.init(args=args)
    node = ExperimentManagerNode()
    try:
        # Run the experiment
        node.run_experiment()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def control_main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def logger_main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv_writer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    simple_test_main()
