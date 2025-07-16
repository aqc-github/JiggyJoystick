#!/usr/bin/env python3
"""
JiggyJoystick System Health Monitor
Comprehensive debugging and monitoring tool for ROS2 nodes, topics, and system health
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32
from custom_interfaces.action import TrialAction
from custom_interfaces.srv import LoadConfig
import time
import json
import psutil
import threading
from collections import defaultdict, deque
from datetime import datetime
import numpy as np
import signal
import sys
import os

class SystemMonitor(Node):
    """
    Comprehensive system monitoring node for JiggyJoystick
    """
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Configuration
        self.monitoring_active = True
        self.data_history_size = 100
        self.health_check_interval = 1.0  # seconds
        
        # Data storage
        self.topic_data = defaultdict(lambda: deque(maxlen=self.data_history_size))
        self.topic_stats = defaultdict(lambda: {
            'message_count': 0,
            'last_message_time': None,
            'frequency': 0.0,
            'average_frequency': 0.0,
            'message_times': deque(maxlen=50)
        })
        
        # System health data
        self.node_health = {}
        self.topic_health = {}
        self.system_stats = {}
        
        # Initialize subscribers for all JiggyJoystick topics
        self.setup_subscribers()
        
        # Health check timer
        self.health_timer = self.create_timer(self.health_check_interval, self.health_check_callback)
        
        # Display timer
        self.display_timer = self.create_timer(2.0, self.display_status)
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info('System Monitor initialized')
        
    def setup_subscribers(self):
        """Setup subscribers for all JiggyJoystick topics"""
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Joint states from Teensy
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', 
            lambda msg: self.topic_callback('/joint_states', msg),
            sensor_qos
        )
        
        # Torque commands
        self.torque_sub = self.create_subscription(
            Float64MultiArray, '/torque_commands',
            lambda msg: self.topic_callback('/torque_commands', msg),
            10
        )
        
        # Experiment control topics
        self.assay_sub = self.create_subscription(
            Int32, '/assay_number',
            lambda msg: self.topic_callback('/assay_number', msg),
            10
        )
        
        self.trial_sub = self.create_subscription(
            Int32, '/trial_number',
            lambda msg: self.topic_callback('/trial_number', msg),
            10
        )
        
        self.trial_status_sub = self.create_subscription(
            Int32, '/trial_status',
            lambda msg: self.topic_callback('/trial_status', msg),
            10
        )
        
        self.trial_success_sub = self.create_subscription(
            Bool, '/trial_success',
            lambda msg: self.topic_callback('/trial_success', msg),
            10
        )
        
        # Trial control topics
        self.start_trial_sub = self.create_subscription(
            Bool, '/start_trial',
            lambda msg: self.topic_callback('/start_trial', msg),
            10
        )
        
        self.abort_trial_sub = self.create_subscription(
            Bool, '/abort_trial',
            lambda msg: self.topic_callback('/abort_trial', msg),
            10
        )
        
        # Handshake topics
        self.ros2_handshake_sub = self.create_subscription(
            Bool, '/ros2_handshake',
            lambda msg: self.topic_callback('/ros2_handshake', msg),
            10
        )
        
        self.microcontroller_handshake_sub = self.create_subscription(
            Bool, '/microcontroller_handshake',
            lambda msg: self.topic_callback('/microcontroller_handshake', msg),
            10
        )
        
    def topic_callback(self, topic_name, msg):
        """Generic callback for all topics"""
        current_time = time.time()
        
        # Store message data
        message_data = {
            'timestamp': current_time,
            'message': self.msg_to_dict(msg)
        }
        self.topic_data[topic_name].append(message_data)
        
        # Update statistics
        stats = self.topic_stats[topic_name]
        stats['message_count'] += 1
        stats['last_message_time'] = current_time
        stats['message_times'].append(current_time)
        
        # Calculate frequency
        if len(stats['message_times']) >= 2:
            time_diffs = [
                stats['message_times'][i] - stats['message_times'][i-1]
                for i in range(1, len(stats['message_times']))
            ]
            if time_diffs:
                avg_interval = sum(time_diffs) / len(time_diffs)
                stats['frequency'] = 1.0 / avg_interval if avg_interval > 0 else 0
        
    def msg_to_dict(self, msg):
        """Convert ROS message to dictionary"""
        if hasattr(msg, 'get_fields_and_field_types'):
            result = {}
            for field_name, field_type in msg.get_fields_and_field_types().items():
                field_value = getattr(msg, field_name)
                if hasattr(field_value, 'get_fields_and_field_types'):
                    result[field_name] = self.msg_to_dict(field_value)
                else:
                    result[field_name] = field_value
            return result
        else:
            return str(msg)
    
    def health_check_callback(self):
        """Perform system health checks"""
        current_time = time.time()
        
        # Check topic health
        for topic_name, stats in self.topic_stats.items():
            if stats['last_message_time']:
                time_since_last = current_time - stats['last_message_time']
                
                # Define expected frequencies for different topics
                expected_frequencies = {
                    '/joint_states': 50.0,  # Should be ~50Hz
                    '/torque_commands': 50.0,  # Should be ~50Hz
                    '/assay_number': 0.1,  # Occasional
                    '/trial_number': 0.1,  # Occasional
                    '/trial_status': 1.0,  # Periodic
                    '/start_trial': 0.1,  # Occasional
                    '/abort_trial': 0.1,  # Occasional
                    '/ros2_handshake': 0.1,  # Occasional
                    '/microcontroller_handshake': 0.1  # Occasional
                }
                
                expected_freq = expected_frequencies.get(topic_name, 1.0)
                max_interval = 3.0 / expected_freq if expected_freq > 0 else 30.0
                
                if time_since_last > max_interval:
                    self.topic_health[topic_name] = {
                        'status': 'WARNING',
                        'message': f'No messages for {time_since_last:.1f}s'
                    }
                else:
                    self.topic_health[topic_name] = {
                        'status': 'OK',
                        'message': f'Active ({stats["frequency"]:.1f} Hz)'
                    }
            else:
                self.topic_health[topic_name] = {
                    'status': 'ERROR',
                    'message': 'No messages received'
                }
        
        # Check system resources
        self.system_stats = {
            'cpu_percent': psutil.cpu_percent(),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_usage': psutil.disk_usage('/').percent,
            'load_average': os.getloadavg()[0] if hasattr(os, 'getloadavg') else 0.0
        }
        
        # Check nodes (simplified check based on topic activity)
        self.check_node_health()
        
    def check_node_health(self):
        """Check health of individual nodes based on their topic activity"""
        current_time = time.time()
        
        # Expected topics for each node
        node_topics = {
            'experiment_manager_node': [],  # Doesn't publish frequently
            'control_node': [
                '/torque_commands', '/assay_number', '/trial_number',
                '/trial_status', '/start_trial', '/abort_trial', '/ros2_handshake'
            ],
            'logger_node': [],  # Doesn't publish, only subscribes
            'micro_ros_simulator': ['/joint_states', '/microcontroller_handshake']
        }
        
        for node_name, expected_topics in node_topics.items():
            if not expected_topics:
                # For nodes without expected topics, assume OK
                self.node_health[node_name] = {
                    'status': 'OK',
                    'message': 'Node assumed active'
                }
                continue
                
            active_topics = 0
            for topic in expected_topics:
                if topic in self.topic_health:
                    if self.topic_health[topic]['status'] == 'OK':
                        active_topics += 1
            
            if active_topics >= len(expected_topics) * 0.5:  # At least 50% of topics active
                self.node_health[node_name] = {
                    'status': 'OK',
                    'message': f'{active_topics}/{len(expected_topics)} topics active'
                }
            else:
                self.node_health[node_name] = {
                    'status': 'WARNING',
                    'message': f'Only {active_topics}/{len(expected_topics)} topics active'
                }
    
    def display_status(self):
        """Display system status"""
        os.system('clear')
        print("=" * 80)
        print("🔧 JIGGYJOYSTICK SYSTEM MONITOR")
        print("=" * 80)
        print(f"📅 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()
        
        # System resources
        print("🖥️  SYSTEM RESOURCES")
        print("-" * 40)
        print(f"CPU Usage:    {self.system_stats.get('cpu_percent', 0):.1f}%")
        print(f"Memory Usage: {self.system_stats.get('memory_percent', 0):.1f}%")
        print(f"Disk Usage:   {self.system_stats.get('disk_usage', 0):.1f}%")
        print(f"Load Average: {self.system_stats.get('load_average', 0):.2f}")
        print()
        
        # Node health
        print("🤖 NODE HEALTH")
        print("-" * 40)
        for node_name, health in self.node_health.items():
            status_emoji = "✅" if health['status'] == 'OK' else "⚠️" if health['status'] == 'WARNING' else "❌"
            print(f"{status_emoji} {node_name:<25} {health['message']}")
        print()
        
        # Topic health
        print("📡 TOPIC HEALTH")
        print("-" * 40)
        for topic_name, health in self.topic_health.items():
            status_emoji = "✅" if health['status'] == 'OK' else "⚠️" if health['status'] == 'WARNING' else "❌"
            print(f"{status_emoji} {topic_name:<30} {health['message']}")
        print()
        
        # Recent data summary
        print("📊 RECENT DATA SUMMARY")
        print("-" * 40)
        
        # Joint states
        if '/joint_states' in self.topic_data and self.topic_data['/joint_states']:
            latest_joint = self.topic_data['/joint_states'][-1]['message']
            if 'position' in latest_joint:
                positions = latest_joint['position']
                print(f"Joint Positions: {positions}")
            if 'velocity' in latest_joint:
                velocities = latest_joint['velocity']
                print(f"Joint Velocities: {velocities}")
        
        # Current trial info
        current_assay = self.get_latest_value('/assay_number')
        current_trial = self.get_latest_value('/trial_number')
        trial_status = self.get_latest_value('/trial_status')
        
        if current_assay is not None:
            print(f"Current Assay: {current_assay}")
        if current_trial is not None:
            print(f"Current Trial: {current_trial}")
        if trial_status is not None:
            status_map = {0: 'Setup Ongoing', 1: 'Setup Successful', -1: 'Aborted', 2: 'Completed'}
            print(f"Trial Status: {status_map.get(trial_status, 'Unknown')}")
        
        print()
        print("Press Ctrl+C to exit")
        
    def get_latest_value(self, topic_name):
        """Get the latest value from a topic"""
        if topic_name in self.topic_data and self.topic_data[topic_name]:
            return self.topic_data[topic_name][-1]['message'].get('data')
        return None
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info('Shutting down System Monitor...')
        self.monitoring_active = False
        sys.exit(0)
        
    def get_diagnostics_report(self):
        """Generate a comprehensive diagnostics report"""
        report = {
            'timestamp': datetime.now().isoformat(),
            'system_stats': self.system_stats,
            'node_health': self.node_health,
            'topic_health': self.topic_health,
            'topic_stats': {
                name: {
                    'message_count': stats['message_count'],
                    'frequency': stats['frequency'],
                    'last_message_time': stats['last_message_time']
                }
                for name, stats in self.topic_stats.items()
            }
        }
        return report

def main(args=None):
    rclpy.init(args=args)
    monitor = SystemMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
