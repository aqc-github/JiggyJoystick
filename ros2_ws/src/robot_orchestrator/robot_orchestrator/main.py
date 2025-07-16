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
            package_share_dir = get_package_share_directory('robot_orchestrator')
            config_path = os.path.join(package_share_dir, 'config', config_file)
            
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            self.assays = self.config.get('assays', [])
            self.get_logger().info(f'✅ Loaded {len(self.assays)} assays from {config_file}')
            
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


class ControlNode(Node):
    """Node that manages trial execution and robot control"""
    
    def __init__(self):
        super().__init__('control_node')
        
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
        
        # Robot parameters (2-DOF arm)
        self.l1 = 30.0 / 1000  # Link 1 length in meters
        self.l2 = 30.0 / 1000  # Link 2 length in meters
        self.base_distance = 40.0 / 1000  # Base separation in meters
        
        # Control loop timer (100 Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('🎮 Control Node initialized')
        self.get_logger().info('Action server "/trial_action" ready')
    
    def forward_kinematics(self, q1, q2):
        """Calculate forward kinematics for 2-DOF arm"""
        # Convert to radians if needed (assuming already in radians)
        x1 = self.l1 * np.cos(q1)
        y1 = self.l1 * np.sin(q1)
        
        x2 = x1 + self.l2 * np.cos(q1 + q2)
        y2 = y1 + self.l2 * np.sin(q1 + q2)
        
        return np.array([x2, y2])
    
    def jacobian(self, q1, q2):
        """Calculate Jacobian matrix for 2-DOF arm"""
        J = np.zeros((2, 2))
        
        # dx/dq1, dx/dq2
        J[0, 0] = -self.l1 * np.sin(q1) - self.l2 * np.sin(q1 + q2)
        J[0, 1] = -self.l2 * np.sin(q1 + q2)
        
        # dy/dq1, dy/dq2
        J[1, 0] = self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2)
        J[1, 1] = self.l2 * np.cos(q1 + q2)
        
        return J
    
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
            ff_forces = self.force_field_matrix @ ee_vel
        
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
            ff_matrix = goal.force_field_matrix
            self.force_field_matrix = np.array([[ff_matrix[0], ff_matrix[1]], [ff_matrix[2], ff_matrix[3]]])
            self.get_logger().info(f'✨ Force field enabled: {self.force_field_matrix.tolist()}')
        else:
            self.force_field_matrix = np.array([[0.0, 0.0], [0.0, 0.0]])
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
                        ff_forces = self.force_field_matrix @ ee_vel
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
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        
        self.get_logger().info('📊 Logger Node initialized')
        self.get_logger().info(f'Logs will be saved to: {self.log_dir}')
    
    def joint_states_callback(self, msg):
        """Handle joint state messages for logging"""
        self.data['joint_positions'] = msg.position
        self.data['joint_velocities'] = msg.velocity
        self.data['joint_efforts'] = msg.effort
        
        # For now, just log occasionally
        if hasattr(self, 'log_count'):
            self.log_count += 1
        else:
            self.log_count = 1
            
        if self.log_count % 500 == 1:  # Log every 500 messages (~10 seconds)
            self.get_logger().info(f'📊 Logged {self.log_count} joint state messages')


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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    simple_test_main()
