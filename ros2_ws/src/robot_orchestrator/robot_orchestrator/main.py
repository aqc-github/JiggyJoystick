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
        """Run the complete experiment (placeholder for now)"""
        self.get_logger().info('🚀 Starting experiment...')
        
        # For now, just log the configuration
        for i, assay in enumerate(self.assays):
            self.get_logger().info(f'  Assay {i+1}: {assay.get("n_trials", 0)} trials, {assay.get("trial_duration", 0)}s each')
        
        self.get_logger().info('⚠️  Full experiment execution not implemented yet')


class ControlNode(Node):
    """Node that manages trial execution and robot control"""
    
    def __init__(self):
        super().__init__('control_node')
        
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
        self.q = None
        self.dq = None
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('🎮 Control Node initialized')
    
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
    
    def print_status(self):
        """Print current status"""
        status = f"🎮 Control Status:\n"
        status += f"  Handshake: {'✅' if self.handshake_complete else '❌'} {'Complete' if self.handshake_complete else 'Waiting'}\n"
        status += f"  Joint States: {'✅' if self.q is not None else '❌'} {'Receiving' if self.q is not None else 'No data'}\n"
        
        if self.q is not None:
            status += f"  Current positions: [{self.q[0]:.3f}, {self.q[1]:.3f}]\n"
            
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
