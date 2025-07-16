#!/usr/bin/env python3
"""
JiggyJoystick Topic Debugger
Interactive tool for inspecting, recording, and analyzing ROS2 topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32
import time
import json
import csv
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from datetime import datetime
import threading
import argparse
import sys
import os

class TopicDebugger(Node):
    """
    Interactive topic debugging tool for JiggyJoystick
    """
    
    def __init__(self, topics_to_monitor=None):
        super().__init__('topic_debugger')
        
        # Configuration
        self.topics_to_monitor = topics_to_monitor or [
            '/joint_states', '/torque_commands', '/assay_number', '/trial_number',
            '/trial_status', '/trial_success', '/start_trial', '/abort_trial',
            '/ros2_handshake', '/microcontroller_handshake'
        ]
        
        self.recording = False
        self.recorded_data = {}
        self.data_buffers = {}
        self.subscribers = {}
        
        # Initialize data buffers
        for topic in self.topics_to_monitor:
            self.data_buffers[topic] = deque(maxlen=1000)
            self.recorded_data[topic] = []
        
        # Setup subscribers
        self.setup_subscribers()
        
        self.get_logger().info(f'Topic Debugger monitoring: {self.topics_to_monitor}')
    
    def setup_subscribers(self):
        """Setup subscribers for monitored topics"""
        
        # Topic type mapping
        topic_types = {
            '/joint_states': JointState,
            '/torque_commands': Float64MultiArray,
            '/assay_number': Int32,
            '/trial_number': Int32,
            '/trial_status': Int32,
            '/trial_success': Bool,
            '/start_trial': Bool,
            '/abort_trial': Bool,
            '/ros2_handshake': Bool,
            '/microcontroller_handshake': Bool
        }
        
        for topic in self.topics_to_monitor:
            msg_type = topic_types.get(topic)
            if msg_type:
                self.subscribers[topic] = self.create_subscription(
                    msg_type, topic,
                    lambda msg, t=topic: self.topic_callback(t, msg),
                    10
                )
    
    def topic_callback(self, topic_name, msg):
        """Generic callback for all monitored topics"""
        timestamp = time.time()
        
        # Convert message to dictionary
        msg_dict = self.msg_to_dict(msg)
        
        # Store in buffer
        data_point = {
            'timestamp': timestamp,
            'data': msg_dict
        }
        
        self.data_buffers[topic_name].append(data_point)
        
        # If recording, also store in recorded data
        if self.recording:
            self.recorded_data[topic_name].append(data_point)
    
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
            return {'data': getattr(msg, 'data', str(msg))}
    
    def start_recording(self):
        """Start recording data"""
        self.recording = True
        # Clear previous recorded data
        for topic in self.topics_to_monitor:
            self.recorded_data[topic] = []
        self.get_logger().info('Started recording')
    
    def stop_recording(self):
        """Stop recording data"""
        self.recording = False
        self.get_logger().info('Stopped recording')
    
    def save_recorded_data(self, filename=None):
        """Save recorded data to file"""
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'topic_debug_{timestamp}.json'
        
        output_data = {}
        for topic, data in self.recorded_data.items():
            if data:  # Only save topics that have data
                output_data[topic] = data
        
        with open(filename, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        self.get_logger().info(f'Saved recorded data to {filename}')
        return filename
    
    def export_to_csv(self, topic_name, filename=None):
        """Export specific topic data to CSV"""
        if topic_name not in self.recorded_data:
            self.get_logger().error(f'Topic {topic_name} not found in recorded data')
            return
        
        data = self.recorded_data[topic_name]
        if not data:
            self.get_logger().error(f'No data recorded for topic {topic_name}')
            return
        
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            safe_topic = topic_name.replace('/', '_')
            filename = f'topic_debug_{safe_topic}_{timestamp}.csv'
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Write header
            if data:
                sample_data = data[0]['data']
                if isinstance(sample_data, dict):
                    header = ['timestamp'] + list(sample_data.keys())
                else:
                    header = ['timestamp', 'data']
                writer.writerow(header)
                
                # Write data
                for point in data:
                    row = [point['timestamp']]
                    if isinstance(point['data'], dict):
                        row.extend(point['data'].values())
                    else:
                        row.append(point['data'])
                    writer.writerow(row)
        
        self.get_logger().info(f'Exported {topic_name} data to {filename}')
        return filename
    
    def get_topic_stats(self, topic_name):
        """Get statistics for a specific topic"""
        if topic_name not in self.data_buffers:
            return None
        
        data = list(self.data_buffers[topic_name])
        if not data:
            return None
        
        # Calculate frequency
        if len(data) > 1:
            time_diffs = [
                data[i]['timestamp'] - data[i-1]['timestamp']
                for i in range(1, len(data))
            ]
            avg_interval = sum(time_diffs) / len(time_diffs)
            frequency = 1.0 / avg_interval if avg_interval > 0 else 0
        else:
            frequency = 0
        
        latest_msg = data[-1]['data'] if data else None
        
        return {
            'message_count': len(data),
            'frequency': frequency,
            'latest_message': latest_msg,
            'first_timestamp': data[0]['timestamp'] if data else None,
            'last_timestamp': data[-1]['timestamp'] if data else None
        }
    
    def plot_topic_data(self, topic_name, field_name=None, save_plot=False):
        """Plot data from a specific topic"""
        if topic_name not in self.recorded_data:
            self.get_logger().error(f'Topic {topic_name} not found in recorded data')
            return
        
        data = self.recorded_data[topic_name]
        if not data:
            self.get_logger().error(f'No data recorded for topic {topic_name}')
            return
        
        # Extract timestamps and values
        timestamps = [point['timestamp'] for point in data]
        start_time = timestamps[0]
        times = [(t - start_time) for t in timestamps]
        
        plt.figure(figsize=(12, 6))
        
        if field_name:
            # Plot specific field
            try:
                values = [point['data'][field_name] for point in data]
                plt.plot(times, values, label=f'{topic_name}.{field_name}')
                plt.ylabel(field_name)
            except KeyError:
                self.get_logger().error(f'Field {field_name} not found in topic data')
                return
        else:
            # Auto-plot numeric fields
            sample_data = data[0]['data']
            if isinstance(sample_data, dict):
                for key, value in sample_data.items():
                    if isinstance(value, (int, float)):
                        values = [point['data'][key] for point in data]
                        plt.plot(times, values, label=f'{topic_name}.{key}')
                    elif isinstance(value, (list, tuple)) and len(value) > 0:
                        # Handle array data
                        for i, _ in enumerate(value):
                            values = [point['data'][key][i] for point in data]
                            plt.plot(times, values, label=f'{topic_name}.{key}[{i}]')
                plt.ylabel('Value')
            else:
                # Simple data
                values = [point['data'] for point in data]
                plt.plot(times, values, label=topic_name)
                plt.ylabel('Value')
        
        plt.xlabel('Time (seconds)')
        plt.title(f'Topic Data: {topic_name}')
        plt.legend()
        plt.grid(True)
        
        if save_plot:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            safe_topic = topic_name.replace('/', '_')
            filename = f'topic_plot_{safe_topic}_{timestamp}.png'
            plt.savefig(filename)
            self.get_logger().info(f'Saved plot to {filename}')
        
        plt.show()
    
    def print_topic_summary(self):
        """Print summary of all monitored topics"""
        print("=" * 80)
        print("📊 TOPIC SUMMARY")
        print("=" * 80)
        
        for topic_name in self.topics_to_monitor:
            stats = self.get_topic_stats(topic_name)
            if stats:
                print(f"\n📡 {topic_name}")
                print(f"   Messages: {stats['message_count']}")
                print(f"   Frequency: {stats['frequency']:.2f} Hz")
                print(f"   Latest: {stats['latest_message']}")
            else:
                print(f"\n📡 {topic_name}")
                print("   No data received")
    
    def interactive_mode(self):
        """Run interactive debugging mode"""
        print("\n🔧 JiggyJoystick Topic Debugger - Interactive Mode")
        print("Available commands:")
        print("  status - Show topic status")
        print("  record - Start recording")
        print("  stop - Stop recording")
        print("  save [filename] - Save recorded data")
        print("  export <topic> [filename] - Export topic to CSV")
        print("  plot <topic> [field] - Plot topic data")
        print("  stats <topic> - Show topic statistics")
        print("  list - List monitored topics")
        print("  quit - Exit")
        
        while True:
            try:
                command = input("\n> ").strip().split()
                if not command:
                    continue
                
                cmd = command[0].lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'status':
                    self.print_topic_summary()
                elif cmd == 'record':
                    self.start_recording()
                elif cmd == 'stop':
                    self.stop_recording()
                elif cmd == 'save':
                    filename = command[1] if len(command) > 1 else None
                    self.save_recorded_data(filename)
                elif cmd == 'export':
                    if len(command) < 2:
                        print("Usage: export <topic> [filename]")
                        continue
                    topic = command[1]
                    filename = command[2] if len(command) > 2 else None
                    self.export_to_csv(topic, filename)
                elif cmd == 'plot':
                    if len(command) < 2:
                        print("Usage: plot <topic> [field]")
                        continue
                    topic = command[1]
                    field = command[2] if len(command) > 2 else None
                    self.plot_topic_data(topic, field, save_plot=True)
                elif cmd == 'stats':
                    if len(command) < 2:
                        print("Usage: stats <topic>")
                        continue
                    topic = command[1]
                    stats = self.get_topic_stats(topic)
                    if stats:
                        print(f"\nStatistics for {topic}:")
                        for key, value in stats.items():
                            print(f"  {key}: {value}")
                    else:
                        print(f"No data available for {topic}")
                elif cmd == 'list':
                    print("\nMonitored topics:")
                    for topic in self.topics_to_monitor:
                        print(f"  - {topic}")
                else:
                    print("Unknown command. Type 'quit' to exit.")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")

def main():
    parser = argparse.ArgumentParser(description='JiggyJoystick Topic Debugger')
    parser.add_argument('--topics', nargs='+', 
                       help='Specific topics to monitor')
    parser.add_argument('--interactive', action='store_true',
                       help='Run in interactive mode')
    parser.add_argument('--record-time', type=float, default=0,
                       help='Auto-record for specified time (seconds)')
    parser.add_argument('--output', type=str,
                       help='Output filename for recorded data')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    debugger = TopicDebugger(topics_to_monitor=args.topics)
    
    if args.interactive:
        # Run interactive mode in separate thread
        interactive_thread = threading.Thread(target=debugger.interactive_mode)
        interactive_thread.daemon = True
        interactive_thread.start()
        
        try:
            rclpy.spin(debugger)
        except KeyboardInterrupt:
            pass
    elif args.record_time > 0:
        # Auto-record mode
        debugger.start_recording()
        
        try:
            # Spin for the specified time
            start_time = time.time()
            while time.time() - start_time < args.record_time:
                rclpy.spin_once(debugger, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        
        debugger.stop_recording()
        filename = debugger.save_recorded_data(args.output)
        print(f"Recording completed. Data saved to {filename}")
    else:
        # Just monitor and show summary
        try:
            # Run for a short time to collect data
            start_time = time.time()
            while time.time() - start_time < 5.0:
                rclpy.spin_once(debugger, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        
        debugger.print_topic_summary()
    
    debugger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
