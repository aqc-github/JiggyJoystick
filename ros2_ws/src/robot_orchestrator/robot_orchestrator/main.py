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

class ExperimentManagerNode(Node):
    def __init__(self):
        super().__init__('experiment_manager_node')
        self.declare_parameter('config_file', 'assays.yaml')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.load_config(config_file)
        self.current_assay = 0
        self.current_trial = 0
        self.trial_action_client = ActionClient(self, TrialAction, 'trial_action')
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
                goal.duration = assay['trial_duration']
                
                if not self.trial_action_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error('Action server /trial_action not available')
                    self.current_trial += 1
                    continue
                    
                future = self.trial_action_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().status == GoalResponse.ACCEPT:
                    self.get_logger().info(f'Started trial {self.current_trial + 1} of assay {self.current_assay + 1}')
                    result_future = future.result().get_result_async()
                    rclpy.spin_until_future_complete(self, result_future)
                    if result_future.result():
                        result = result_future.result().result
                        status = 'completed' if result.success else 'aborted'
                        self.get_logger().info(f'Trial {self.current_trial + 1} {status}')
                    else:
                        self.get_logger().warn('Trial result not received')
                else:
                    self.get_logger().warn('Trial goal rejected')
                self.current_trial += 1
            self.current_assay += 1
            self.current_trial = 0
        self.get_logger().info('Experiment completed')

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.trial_action_server = ActionServer(
            self, TrialAction, 'trial_action', self.execute_trial_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.torque_pub = self.create_publisher(Float64MultiArray, '/torque_commands', 10)
        self.assay_pub = self.create_publisher(Int32, '/assay_number', 10)
        self.trial_pub = self.create_publisher(Int32, '/trial_number', 10)
        self.trial_status_pub = self.create_publisher(Int32, '/trial_status', 10)
        self.trial_success_pub = self.create_publisher(Bool, '/trial_success', 10)
        self.start_trial_pub = self.create_publisher(Bool, '/start_trial', 10)
        self.abort_trial_pub = self.create_publisher(Bool, '/abort_trial', 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.trial_status_sub = self.create_subscription(
            Int32, '/trial_status', self.trial_status_callback, 10
        )
        # Handshake topics updated to match thinking trace
        self.handshake_pub = self.create_publisher(Bool, '/ros2_handshake', 10)
        self.handshake_sub = self.create_subscription(
            Bool, '/microcontroller_handshake', self.handshake_callback, 10
        )
        self.handshake_complete = False  # Tracks handshake status
        self.timer = self.create_timer(0.01, self.control_loop)
        self.current_goal = None
        self.trial_start_time = None
        self.trial_duration = 0.0
        self.q = None
        self.dq = None
        self.trial_status = 0
        self.target_pos = np.array([0.0, 0.0])
        self.tolerance = 1.0
        self.target_angles = np.array([0.0, 0.0])
        self.angle_tolerance = 0.01
        self.l1 = 30.0 / 1000  # Convert mm to m
        self.l2 = 30.0 / 1000
        self.base_distance = 40.0 / 1000
        self.get_logger().info('Control Node initialized')

    def joint_state_callback(self, msg):
        self.q = np.array(msg.position)
        self.dq = np.array(msg.velocity)

    def trial_status_callback(self, msg):
        self.trial_status = msg.data

    def goal_callback(self, goal_request):
        """Reject goals if handshake is not complete."""
        if not self.handshake_complete:
            self.get_logger().warn('Handshake not complete, rejecting goal')
            return GoalResponse.REJECT
        self.get_logger().info('Goal accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Trial cancellation requested')
        self.abort_trial_pub.publish(Bool(data=True))
        self.trial_success_pub.publish(Bool(data=False))
        return CancelResponse.ACCEPT

    def handshake_callback(self, msg):
        """Handle handshake from microcontroller."""
        if msg.data and not self.handshake_complete:
            self.get_logger().info('Received handshake from microcontroller')
            self.handshake_pub.publish(Bool(data=True))
            self.handshake_complete = True
            self.get_logger().info('Handshake complete, sent response')

    def execute_trial_callback(self, goal_handle):
        goal = goal_handle.request
        self.current_goal = goal_handle
        self.trial_start_time = self.get_clock().now()
        self.trial_duration = goal.duration
        self.get_logger().info(f'Executing trial {goal.trial_number} of assay {goal.assay_number}')
        self.assay_pub.publish(Int32(data=goal.assay_number))
        self.trial_pub.publish(Int32(data=goal.trial_number))
        self.trial_status_pub.publish(Int32(data=1))  # Setup successful
        self.start_trial_pub.publish(Bool(data=True))  # Start trial

        result = TrialAction.Result()
        while rclpy.ok() and self.current_goal == goal_handle:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Trial canceled by client')
                self.abort_trial_pub.publish(Bool(data=True))
                self.trial_success_pub.publish(Bool(data=False))
                goal_handle.canceled()
                result.success = False
                return result
            time_elapsed = (self.get_clock().now() - self.trial_start_time).nanoseconds / 1e9
            if time_elapsed >= self.trial_duration or self.trial_status == -1:
                if self.trial_status == -1:
                    self.get_logger().info('Trial aborted due to external signal')
                    self.abort_trial_pub.publish(Bool(data=True))
                    self.trial_success_pub.publish(Bool(data=False))
                    result.success = False
                else:
                    current_pos = self.task_state()
                    distance = np.linalg.norm(current_pos - self.target_pos)
                    if self.q is not None:
                        angle_diffs = np.abs(self.q - self.target_angles)
                        success = bool((distance < self.tolerance) and np.all(angle_diffs < self.angle_tolerance))
                    else:
                        success = False
                        angle_diffs = np.array([float('inf'), float('inf')])
                    self.get_logger().info(f'Trial ended, distance: {distance:.4f}, angle diffs: {angle_diffs}, success: {success}')
                    self.trial_success_pub.publish(Bool(data=success))
                    self.trial_status_pub.publish(Int32(data=2 if success else -1))
                    self.abort_trial_pub.publish(Bool(data=True))
                    result.success = success
                goal_handle.succeed() if result.success else goal_handle.abort()
                self.current_goal = None
                return result
            feedback = TrialAction.Feedback()
            if self.q is not None and self.dq is not None:
                feedback.joint_positions = self.q.tolist()
                feedback.joint_velocities = self.dq.tolist()
                tau = np.zeros(2)
                feedback.torques = tau.tolist()
                goal_handle.publish_feedback(feedback)
            time.sleep(0.01)
        result.success = False
        self.trial_success_pub.publish(Bool(data=False))
        self.abort_trial_pub.publish(Bool(data=True))
        goal_handle.abort()
        return result

    def control_loop(self):
        if not hasattr(self, 'q') or self.current_goal is None:
            self.torque_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
            return
        tau = np.zeros(2)
        self.torque_pub.publish(Float64MultiArray(data=tau.tolist()))

    def task_state(self):
        if self.q is None:
            return np.array([0.0, 0.0])
        q1, q2 = self.q
        P1x = self.l1 * np.cos(q1)
        P1y = self.l1 * np.sin(q1)
        P2x = self.base_distance + self.l1 * np.cos(q2)
        P2y = self.l1 * np.sin(q2)
        Vx = P2x - P1x
        Vy = P2y - P1y
        D = np.sqrt(Vx**2 + Vy**2)
        if D > 2 * self.l2:
            return np.array([0.0, 0.0])
        midpoint_x = (P1x + P2x) / 2
        midpoint_y = (P1y + P2y) / 2
        Ux = Vx / D
        Uy = Vy / D
        U_perp_x = -Uy
        U_perp_y = Ux
        h = np.sqrt(self.l2**2 - (D / 2)**2)
        intersection1_x = midpoint_x + h * U_perp_x
        intersection1_y = midpoint_y + h * U_perp_y
        intersection2_x = midpoint_x - h * U_perp_x
        intersection2_y = midpoint_y - h * U_perp_y
        if intersection1_y > intersection2_y:
            return np.array([intersection1_x, intersection1_y])
        return np.array([intersection2_x, intersection2_y])

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
        self.trial_success_sub = self.create_subscription(Bool, '/trial_success', self.trial_success_callback, 10)
        self.data = {}

    def open_csv(self, assay, trial):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'log_assay{assay}_trial{trial}_{timestamp}.csv'
        filepath = os.path.join(self.log_dir, filename)
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'assay', 'trial', 'joint_positions', 'joint_velocities', 
                                 'torques', 'trial_status', 'trial_success'])

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

    def trial_success_callback(self, msg):
        self.data['trial_success'] = msg.data
        self.get_logger().info(f'Trial success: {msg.data}')

    def log_data(self):
        if self.csv_writer:
            self.csv_writer.writerow([
                datetime.now().isoformat(),
                self.data.get('assay'),
                self.data.get('trial'),
                self.data.get('joint_positions'),
                self.data.get('joint_velocities'),
                self.data.get('torques'),
                self.data.get('trial_status'),
                self.data.get('trial_success')
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
    pass