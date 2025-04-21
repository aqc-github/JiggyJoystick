#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import time
import csv
import os
from datetime import datetime

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        # Parameters
        self.declare_parameter('config_file', 'assays.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.load_config(config_file)

        # Robot parameters (2-DoF arm)
        self.l1, self.l2 = 0.5, 0.5  # Link lengths (m)
        self.current_assay = 0
        self.current_trial = 0
        self.trial_status = 0
        self.trial_start_time = self.get_clock().now()

        # Publishers
        self.torque_pub = self.create_publisher(Float64MultiArray, '/torque_commands', 10)
        self.assay_pub = self.create_publisher(Int32, '/assay_number', 10)
        self.trial_pub = self.create_publisher(Int32, '/trial_number', 10)
        self.abort_trial_pub = self.create_publisher(Bool, '/abort_trial', 10)
        self.ff_enabled_pub = self.create_publisher(Bool, '/ff_enabled', 10)
        self.ff_value_pub = self.create_publisher(Float64MultiArray, '/ff_value', 10)

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.trial_status_sub = self.create_subscription(
            Int32, '/trial_status', self.trial_status_callback, 10)

        # Timer for control loop (100 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)

    def load_config(self, config_file):
        """
        Load configuration file and initialize variables.

        Args:
            config_file (str): The path to the configuration file.
        """
        package_share_dir = get_package_share_directory('robot_system')
        config_path = os.path.join(package_share_dir, 'config', config_file)
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.assays = self.config['assays']
        self.get_logger().info(f'Loaded {len(self.assays)} assays')

    def joint_state_callback(self, msg):
        self.q = np.array(msg.position)  # Joint angles [q1, q2]
        self.dq = np.array(msg.velocity)  # Joint velocities [dq1, dq2]

    def trial_status_callback(self, msg):
        """
        Callback function for trial status updates. Depending on the status code, log the corresponding message.

        Args:
            msg (Int32): The trial status message.
        """
        if msg.data == 0:
            self.get_logger().info('Trial setup ongoing')
        elif msg.data == 1:
            self.get_logger().info('Trial setup successful -> Execute trial')
        elif msg.data == -1:
            self.get_logger().error('Trial aborted -> Reset trial')
        elif msg.data == 2:
            self.get_logger().info('Trial completed')
        else:
            self.get_logger().info('Trial ongoing')

        self.trial_status = msg.data

    def compute_jacobian(self):
        J = np.array([
            [-self.l1 * np.sin(self.q[0]) - self.l2 * np.sin(self.q[0] + self.q[1]), -self.l2 * np.sin(self.q[0] + self.q[1])],
            [self.l1 * np.cos(self.q[0]) + self.l2 * np.cos(self.q[0] + self.q[1]), self.l2 * np.cos(self.q[0] + self.q[1])]
        ])
        return J

    def task_state(self):
        q1, q2 = self.q
        x = self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2)
        y = self.l1 * np.sin(q1) + self.l2 * np.sin(q1 + q2)
        return np.array([x, y])

    def control_loop(self):
        if not hasattr(self, 'q'):
            return  # Wait for joint state to start
        if self.current_assay >= len(self.assays):
            self.get_logger().info('All assays completed')
            self.timer.destroy()
            return

        # Load the current assay configuration
        assay = self.assays[self.current_assay]
        n_trials = assay['n_trials']
        ff_enabled = assay['force_field']['enabled']
        ff_matrix = np.array(assay['force_field']['matrix']).reshape(2, 2)
        trial_duration = assay['trial_duration']  # seconds

        # Check if trial is complete or aborted
        current_time = self.get_clock().now()
        time_elapsed = (current_time - self.trial_start_time).nanoseconds / 1e9
        if time_elapsed >= trial_duration or self.trial_status == -1:
            if self.trial_status == -1:
                self.get_logger().info(f'Trial {self.current_trial + 1} aborted')
                self.abort_trial_pub.publish(Bool(data=True))
            else:
                self.get_logger().info(f'Trial {self.current_trial + 1} completed')
            self.current_trial += 1
            self.trial_start_time = current_time
            self.trial_status = 0  # Reset status for next trial
            if self.current_trial >= n_trials:
                self.current_assay += 1
                self.current_trial = 0
                self.get_logger().info(f'Starting assay {self.current_assay + 1}')

        # Compute force field
        task_pos = self.task_state()
        F = np.zeros(2)
        if ff_enabled:
            F = ff_matrix @ task_pos

        # Compute torques
        J = self.compute_jacobian()
        tau = J.T @ F

        # Publish data
        torque_msg = Float64MultiArray(data=tau.tolist())
        assay_msg = Int32(data=self.current_assay + 1)
        trial_msg = Int32(data=self.current_trial + 1)
        ff_enabled_msg = Bool(data=ff_enabled)
        ff_value_msg = Float64MultiArray(data=ff_matrix.flatten().tolist())

        self.torque_pub.publish(torque_msg)
        self.assay_pub.publish(assay_msg)
        self.trial_pub.publish(trial_msg)
        self.ff_enabled_pub.publish(ff_enabled_msg)
        self.ff_value_pub.publish(ff_value_msg)

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.declare_parameter('log_file', 'experiment_log.csv')
        self.log_file = self.get_parameter('log_file').get_parameter_value().string_value

        # Initialize CSV file
        self.init_csv()

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.torque_sub = self.create_subscription(
            Float64MultiArray, '/torque_commands', self.torque_callback, 10)
        self.assay_sub = self.create_subscription(
            Int32, '/assay_number', self.assay_callback, 10)
        self.trial_sub = self.create_subscription(
            Int32, '/trial_number', self.trial_callback, 10)
        self.ff_enabled_sub = self.create_subscription(
            Bool, '/ff_enabled', self.ff_enabled_callback, 10)
        self.ff_value_sub = self.create_subscription(
            Float64MultiArray, '/ff_value', self.ff_value_callback, 10)
        self.trial_status_sub = self.create_subscription(
            Int32, '/trial_status', self.trial_status_callback, 10)

        # Data storage
        self.task_state = None
        self.torque = None
        self.assay_number = 0
        self.trial_number = 0
        self.ff_enabled = False
        self.ff_value = [0.0, 0.0, 0.0, 0.0]
        self.trial_status = 0

        # Timer for logging (10 Hz)
        self.timer = self.create_timer(0.1, self.log_data)

    def init_csv(self):
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'assay_number', 'trial_number', 'trial_status', 'ff_enabled',
                    'ff_m11', 'ff_m12', 'ff_m21', 'ff_m22',
                    'task_x', 'task_y', 'torque_1', 'torque_2'
                ])

    def joint_state_callback(self, msg):
        q1, q2 = msg.position
        l1, l2 = 0.5, 0.5
        x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
        y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
        self.task_state = [x, y]

    def torque_callback(self, msg):
        self.torque = msg.data

    def assay_callback(self, msg):
        self.assay_number = msg.data

    def trial_callback(self, msg):
        self.trial_number = msg.data

    def ff_enabled_callback(self, msg):
        self.ff_enabled = msg.data

    def ff_value_callback(self, msg):
        self.ff_value = msg.data

    def trial_status_callback(self, msg):
        self.trial_status = msg.data

    def log_data(self):
        if self.task_state is None or self.torque is None:
            return

        timestamp = time.time()
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp, self.assay_number, self.trial_number, self.trial_status, self.ff_enabled,
                self.ff_value[0], self.ff_value[1], self.ff_value[2], self.ff_value[3],
                self.task_state[0], self.task_state[1], self.torque[0], self.torque[1]
            ])

def control_main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

def logger_main(args=None):
    rclpy.init(args=args)
    logger_node = LoggerNode()
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'control':
        control_main()
    elif len(sys.argv) > 1 and sys.argv[1] == 'logger':
        logger_main()
    else:
        print("Usage: ros2 run robot_system robot_system -- control|logger")
