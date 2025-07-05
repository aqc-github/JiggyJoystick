#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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

class ExperimentManagerNode(Node):
    def __init__(self):
        super().__init__('experiment_manager_node')
        self.declare_parameter('config_file', 'assays.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.load_config(config_file)
        self.current_assay = 0
        self.current_trial = 0
        self.trial_action_client = self.create_client(TrialAction, 'trial_action')
        self.load_config_service = self.create_service(LoadConfig, 'load_config', self.load_config_callback)
        self.get_logger().info('Experiment Manager Node initialized')

    def load_config(self, config_file):
        package_share_dir = get_package_share_directory('robot_orchestrator')
        config_path = os.path.join(package_share_dir, 'config', config_file)
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.assays = self.config['assays']
        self.get_logger().info(f'Loaded {len(self.assays)} assays from {config_file}')

    def load_config_callback(self, request, response):
        try:
            self.load_config(request.config_file_path)
            response.success = True
            response.message = "Configuration loaded successfully"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def run_experiment(self):
        while self.current_assay < len(self.assays):
            assay = self.assays[self.current_assay]
            n_trials = assay['n_trials']
            while self.current_trial < n_trials:
                goal = TrialAction.Goal()
                goal.assay_number = self.current_assay + 1
                goal.trial_number = self.current_trial + 1
                goal.force_field_enabled = assay['force_field']['enabled']
                goal.force_field_matrix = assay['force_field']['matrix']
                goal.duration = assay['trial_duration']
                self.trial_action_client.wait_for_server()
                future = self.trial_action_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().status == GoalResponse.ACCEPT:
                    self.get_logger().info(f'Started trial {self.current_trial + 1} of assay {self.current_assay + 1}')
                    result_future = future.result().get_result_async()
                    rclpy.spin_until_future_complete(self, result_future)
                    result = result_future.result().result
                    status = 'completed' if result.success else 'aborted'
                    self.get_logger().info(f'Trial {self.current_trial + 1} {status}')
                else:
                    self.get_logger().warn('Trial goal rejected')
                self.current_trial += 1
            self.current_assay += 1
            self.current_trial = 0
        self.get_logger().info('Experiment completed')

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.l1, self.l2 = 0.5, 0.5  # Link lengths (m)
        self.trial_action_server = ActionServer(
            self, TrialAction, 'trial_action', self.execute_trial_callback,
            cancel_callback=self.cancel_callback
        )
        self.torque_pub = self.create_publisher(Float64MultiArray, '/torque_commands', 10)
        self.assay_pub = self.create_publisher(Int32, '/assay_number', 10)
        self.trial_pub = self.create_publisher(Int32, '/trial_number', 10)
        self.trial_status_pub = self.create_publisher(Int32, '/trial_status', 10)
        self.ff_enabled_pub = self.create_publisher(Bool, '/ff_enabled', 10)
        self.ff_value_pub = self.create_publisher(Float64MultiArray, '/ff_value', 10)
        self.abort_trial_pub = self.create_publisher(Bool, '/abort_trial', 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.trial_status_sub = self.create_subscription(
            Int32, '/trial_status', self.trial_status_callback, 10
        )
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        self.current_goal = None
        self.trial_start_time = None
        self.trial_duration = 0.0
        self.ff_enabled = False
        self.ff_matrix = np.zeros((2, 2))
        self.trial_status = 0
        self.q = None
        self.dq = None
        self.get_logger().info('Control Node initialized')

    def joint_state_callback(self, msg):
        self.q = np.array(msg.position)  # Joint angles [q1, q2]
        self.dq = np.array(msg.velocity)  # Joint velocities [dq1, dq2]

    def trial_status_callback(self, msg):
        self.trial_status = msg.data

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Trial cancellation requested')
        self.abort_trial_pub.publish(Bool(data=True))
        return CancelResponse.ACCEPT

    def execute_trial_callback(self, goal_handle):
        goal = goal_handle.request
        self.current_goal = goal_handle
        self.trial_start_time = self.get_clock().now()
        self.trial_duration = goal.duration
        self.ff_enabled = goal.force_field_enabled
        self.ff_matrix = np.array(goal.force_field_matrix).reshape(2, 2)
        self.get_logger().info(f'Executing trial {goal.trial_number} of assay {goal.assay_number}')
        self.assay_pub.publish(Int32(data=goal.assay_number))
        self.trial_pub.publish(Int32(data=goal.trial_number))
        self.ff_enabled_pub.publish(Bool(data=self.ff_enabled))
        self.ff_value_pub.publish(Float64MultiArray(data=goal.force_field_matrix))
        self.trial_status_pub.publish(Int32(data=1))  # Setup successful

        result = TrialAction.Result()
        while rclpy.ok() and self.current_goal == goal_handle:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Trial canceled by client')
                self.abort_trial_pub.publish(Bool(data=True))
                goal_handle.canceled()
                result.success = False
                return result
            time_elapsed = (self.get_clock().now() - self.trial_start_time).nanoseconds / 1e9
            if time_elapsed >= self.trial_duration or self.trial_status == -1:
                if self.trial_status == -1:
                    self.get_logger().info('Trial aborted due to external signal')
                    self.abort_trial_pub.publish(Bool(data=True))
                    self.trial_status_pub.publish(Int32(data=-1))
                    result.success = False
                else:
                    self.get_logger().info('Trial completed successfully')
                    self.trial_status_pub.publish(Int32(data=2))
                    result.success = True
                goal_handle.succeed()
                self.current_goal = None
                return result
            feedback = TrialAction.Feedback()
            if self.q is not None and self.dq is not None:
                feedback.joint_positions = self.q.tolist()
                feedback.joint_velocities = self.dq.tolist()
                tau = self.compute_torques()
                feedback.torques = tau.tolist()
                goal_handle.publish_feedback(feedback)
            time.sleep(0.01)
        result.success = False
        goal_handle.abort()
        return result

    def control_loop(self):
        if not hasattr(self, 'q') or self.current_goal is None:
            return
        tau = self.compute_torques()
        self.torque_pub.publish(Float64MultiArray(data=tau.tolist()))

    def compute_torques(self):
        task_pos = self.task_state()
        F = np.zeros(2)
        if self.ff_enabled:
            F = self.ff_matrix @ task_pos
        J = self.compute_jacobian()
        return J.T @ F

    def compute_jacobian(self):
        J = np.array([
            [-self.l1 * np.sin(self.q[0]) - self.l2 * np.sin(self.q[0] + self.q[1]), 
             -self.l2 * np.sin(self.q[0] + self.q[1])],
            [self.l1 * np.cos(self.q[0]) + self.l2 * np.cos(self.q[0] + self.q[1]), 
             self.l2 * np.cos(self.q[0] + self.q[1])]
        ])
        return J

    def task_state(self):
        q1, q2 = self.q
        x = self.l1 * np.cos(q1) + self.l2 * np.cos(q1 + q2)
        y = self.l1 * np.sin(q1) + self.l2 * np.sin(q1 + q2)
        return np.array([x, y])

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.log_dir = os.path.join(os.getcwd(), 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_file = None
        self.csv_writer = None
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.torque_sub = self.create_subscription(Float64MultiArray, '/torque_commands', self.torque_callback, 10)
        self.assay_sub = self.create_subscription(Int32, '/assay_number', self.assay_callback, 10)
        self.trial_sub = self.create_subscription(Int32, '/trial_number', self.trial_callback, 10)
        self.trial_status_sub = self.create_subscription(Int32, '/trial_status', self.trial_status_callback, 10)
        self.ff_enabled_sub = self.create_subscription(Bool, '/ff_enabled', self.ff_enabled_callback, 10)
        self.ff_value_sub = self.create_subscription(Float64MultiArray, '/ff_value', self.ff_value_callback, 10)
        self.data = {}

    def open_csv(self, assay, trial):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'log_assay{assay}_trial{trial}_{timestamp}.csv'
        filepath = os.path.join(self.log_dir, filename)
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'assay', 'trial', 'joint_positions', 'joint_velocities', 
                                 'torques', 'ff_enabled', 'ff_value', 'trial_status'])

    def close_csv(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

    def joint_state_callback(self, msg):
        self.data['joint_positions'] = msg.position
        self.data['joint_velocities'] = msg.velocity
        self.log_data()

    def torque_callback(self, msg):
        self.data['torques'] = msg.data

    def assay_callback(self, msg):
        self.data['assay'] = msg.data

    def trial_callback(self, msg):
        self.data['trial'] = msg.data
        if 'assay' in self.data:
            self.close_csv()
            self.open_csv(self.data['assay'], msg.data)

    def trial_status_callback(self, msg):
        self.data['trial_status'] = msg.data
        status_map = {0: 'setup ongoing', 1: 'setup successful', -1: 'aborted', 2: 'completed'}
        self.get_logger().info(f'Trial status: {status_map.get(msg.data, "ongoing")}')

    def ff_enabled_callback(self, msg):
        self.data['ff_enabled'] = msg.data

    def ff_value_callback(self, msg):
        self.data['ff_value'] = msg.data

    def log_data(self):
        if self.csv_writer:
            self.csv_writer.writerow([
                datetime.now().isoformat(),
                self.data.get('assay'),
                self.data.get('trial'),
                self.data.get('joint_positions'),
                self.data.get('joint_velocities'),
                self.data.get('torques'),
                self.data.get('ff_enabled'),
                self.data.get('ff_value'),
                self.data.get('trial_status')
            ])

def experiment_manager_main(args=None):
    rclpy.init(args=args)
    node = ExperimentManagerNode()
    node.run_experiment()
    rclpy.shutdown()

def control_main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

def logger_main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    # This is not intended to be run as a script
    pass
