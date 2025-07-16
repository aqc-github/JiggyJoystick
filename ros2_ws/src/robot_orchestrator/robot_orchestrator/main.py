#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time


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


# Legacy entry points (kept for compatibility)
def experiment_manager_main(args=None):
    simple_test_main(args)


def control_main(args=None):
    simple_test_main(args)


def logger_main(args=None):
    simple_test_main(args)


if __name__ == '__main__':
    simple_test_main()
