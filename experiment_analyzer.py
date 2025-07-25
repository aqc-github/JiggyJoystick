#!/usr/bin/env python3
"""
JiggyJoystick Experiment Log Analyzer and Visualizer
Analyzes and visualizes experiment logs from the 5-bar parallel robot simulation.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import json
import argparse
from datetime import datetime
from five_bar_kinematics import FiveBarKinematics


class JiggyJoystickAnalyzer:
    def __init__(self, log_directory="/ros2_ws/logs"):
        self.log_directory = log_directory
        # Load robot parameters from experiment configuration
        self.robot_params = self.load_robot_parameters()
        # Initialize 5-bar parallel robot kinematics
        self.kinematics = FiveBarKinematics(
            self.robot_params['L1'], 
            self.robot_params['L2'], 
            self.robot_params['d']
        )

    def load_robot_parameters(self):
        """Load robot parameters from the configuration file"""
        config_path = '/home/albertoquintana/Documentos/JiggyJoystick/ros2_ws/src/robot_orchestrator/config/assays.yaml'
        try:
            with open(config_path, 'r') as conf_file:
                import yaml
                config = yaml.safe_load(conf_file)
                lengths = config['kinematics'][0]['link_length']
                
                # Also load experiment positions (keep in mm for plotting)
                experiment_config = config.get('experiment', {})
                self.start_position = np.array(experiment_config.get('start_position', [40.0, 25.0]))  # Keep in mm
                self.end_position = np.array(experiment_config.get('end_position', [25.0, 25.0]))  # Keep in mm
                
                return {
                    'L1': lengths[0] / 1000.0,  # Convert to meters
                    'L2': lengths[1] / 1000.0,  # Convert to meters
                    'd': lengths[2] / 1000.0,   # Convert to meters
                }
        except Exception as e:
            print(f"Error loading robot parameters: {e}")
            # Default experiment positions (keep in mm)
            self.start_position = np.array([40.0, 25.0])
            self.end_position = np.array([25.0, 25.0])
            return {
                'L1': 0.0635,  # Default values
                'L2': 0.0635,
                'd': 0.0508,
            }
        
    def forward_kinematics(self, theta1, theta2):
        """
        Compute end-effector position from joint angles using proper 5-bar parallel robot kinematics.
        Returns (x, y) position of end-effector.
        """
        position = self.kinematics.direct_kinematics(theta1, theta2)
        return position[0], position[1]
    
    def load_experiment_data(self, pattern="*assay*.csv"):
        """Load all CSV files matching the pattern."""
        csv_files = glob.glob(os.path.join(self.log_directory, pattern))
        csv_files.sort()
        
        experiments = {}
        for file_path in csv_files:
            filename = os.path.basename(file_path)
            # Extract assay number from filename
            assay_num = filename.split('assay')[1].split('.')[0]
            
            try:
                df = pd.read_csv(file_path)
                # Convert string representations to actual lists
                df['joint_positions'] = df['joint_positions'].apply(eval)
                df['joint_velocities'] = df['joint_velocities'].apply(eval) 
                df['joint_efforts'] = df['joint_efforts'].apply(eval)
                df['timestamp'] = pd.to_datetime(df['timestamp'])
                
                experiments[f"Assay {assay_num}"] = df
                print(f"Loaded {filename}: {len(df)} data points")
            except Exception as e:
                print(f"Error loading {filename}: {e}")
                
        return experiments
    
    def compute_end_effector_trajectory(self, df):
        """Compute end-effector trajectory from joint positions."""
        trajectories = []
        
        for _, row in df.iterrows():
            theta1, theta2 = row['joint_positions']
            x_ee, y_ee = self.forward_kinematics(theta1, theta2)
            trajectories.append([x_ee, y_ee])
            
        return np.array(trajectories)
    
    def plot_workspace_boundary(self, ax):
        """Plot the robot's actual workspace boundary"""
        # Generate workspace boundary by sampling joint angles
        boundary_points = []
        
        # Sample the joint space to find workspace boundary
        theta1_range = np.linspace(-0.5, 0.5, 100)
        theta2_range = np.linspace(-0.5, 0.5, 100)
        
        for theta1 in theta1_range:
            for theta2 in [-0.5, 0.5]:  # Extremes of theta2
                pos = self.kinematics.direct_kinematics(theta1, theta2)
                if not np.isnan(pos[0]) and not np.isnan(pos[1]):
                    boundary_points.append(pos * 1000.0)
                    
        for theta2 in theta2_range:
            for theta1 in [-0.5, 0.5]:  # Extremes of theta1
                pos = self.kinematics.direct_kinematics(theta1, theta2)
                if not np.isnan(pos[0]) and not np.isnan(pos[1]):
                    boundary_points.append(pos * 1000.0)
        
        if boundary_points:
            boundary_points = np.array(boundary_points)
            # Plot boundary as scattered points
            ax.scatter(boundary_points[:, 0], boundary_points[:, 1], 
                      c='gray', s=1, alpha=0.3, label='Workspace Boundary')
    
    def analyze_workspace(self):
        """Analyze robot workspace and report on expected positions"""
        print("\nWorkspace Analysis:")
        print("-" * 30)
        
        # Calculate actual workspace limits
        positions = []
        for theta1 in np.linspace(-0.5, 0.5, 50):
            for theta2 in np.linspace(-0.5, 0.5, 50):
                pos = self.kinematics.direct_kinematics(theta1, theta2)
                if not np.isnan(pos[0]) and not np.isnan(pos[1]):
                    positions.append(pos * 1000.0)
        
        if positions:
            positions = np.array(positions)
            print(f"Actual workspace:")
            print(f"  X range: {positions[:, 0].min():.1f} to {positions[:, 0].max():.1f} mm")
            print(f"  Y range: {positions[:, 1].min():.1f} to {positions[:, 1].max():.1f} mm")
            print(f"  Center: ({positions[:, 0].mean():.1f}, {positions[:, 1].mean():.1f}) mm")
        
        # Check expected positions
        start_reachable = self.kinematics.is_position_reachable(self.start_position[0]/1000.0, self.start_position[1]/1000.0)
        end_reachable = self.kinematics.is_position_reachable(self.end_position[0]/1000.0, self.end_position[1]/1000.0)
        
        print(f"\nExpected positions:")
        print(f"  Start position: ({self.start_position[0]:.1f}, {self.start_position[1]:.1f}) mm - {'✅ REACHABLE' if start_reachable else '❌ UNREACHABLE'}")
        print(f"  End position: ({self.end_position[0]:.1f}, {self.end_position[1]:.1f}) mm - {'✅ REACHABLE' if end_reachable else '❌ UNREACHABLE'}")
        
        if not start_reachable or not end_reachable:
            print("\n⚠️  WARNING: Some expected positions are outside the robot's workspace!")
            print("   The experiment configuration may need to be updated with reachable positions.")
            
            # Suggest alternative positions
            print("\n💡 Suggested reachable positions near the center of workspace:")
            if positions is not None and len(positions) > 0:
                center_x = positions[:, 0].mean()
                center_y = positions[:, 1].mean()
                print(f"   Center: ({center_x:.1f}, {center_y:.1f}) mm")
                
                # Find positions at reasonable distance from center
                distances = np.sqrt((positions[:, 0] - center_x)**2 + (positions[:, 1] - center_y)**2)
                target_dist = 10.0  # 10mm from center
                close_indices = np.where(np.abs(distances - target_dist) < 2.0)[0]  # Within 2mm of target
                
                if len(close_indices) > 0:
                    sample_positions = positions[close_indices][:5]  # Take first 5 samples
                    print(f"   Positions ~{target_dist:.0f}mm from center:")
                    for i, pos in enumerate(sample_positions):
                        print(f"     Option {i+1}: ({pos[0]:.1f}, {pos[1]:.1f}) mm")
    
    def plot_joint_analysis(self, experiments):
        """Create comprehensive joint analysis plots."""
        fig, axes = plt.subplots(3, 1, figsize=(15, 12))
        fig.suptitle('Joint Analysis Across All Assays', fontsize=16, fontweight='bold')
        
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        
        for i, (assay_name, df) in enumerate(experiments.items()):
            color = colors[i % len(colors)]
            
            # Extract joint data
            joint_positions = np.array(df['joint_positions'].tolist())
            joint_velocities = np.array(df['joint_velocities'].tolist())
            joint_efforts = np.array(df['joint_efforts'].tolist())
            
            # Convert timestamps to relative time (seconds from start)
            time_rel = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
            
            # Plot joint positions
            axes[0].plot(time_rel, joint_positions[:, 0], 
                        label=f'{assay_name} - Joint 1', color=color, linestyle='-')
            axes[0].plot(time_rel, joint_positions[:, 1], 
                        label=f'{assay_name} - Joint 2', color=color, linestyle='--')
            
            # Plot joint velocities
            axes[1].plot(time_rel, joint_velocities[:, 0], 
                        label=f'{assay_name} - Joint 1', color=color, linestyle='-')
            axes[1].plot(time_rel, joint_velocities[:, 1], 
                        label=f'{assay_name} - Joint 2', color=color, linestyle='--')
            
            # Plot joint efforts
            axes[2].plot(time_rel, joint_efforts[:, 0], 
                        label=f'{assay_name} - Joint 1', color=color, linestyle='-')
            axes[2].plot(time_rel, joint_efforts[:, 1], 
                        label=f'{assay_name} - Joint 2', color=color, linestyle='--')
        
        # Configure plots
        axes[0].set_title('Joint Positions Over Time')
        axes[0].set_ylabel('Position (rad)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        
        axes[1].set_title('Joint Velocities Over Time')
        axes[1].set_ylabel('Velocity (rad/s)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        
        axes[2].set_title('Joint Efforts Over Time')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Effort (Nm)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        
        plt.tight_layout()
        plt.show()
    
    def plot_end_effector_trajectories(self, experiments):
        """Plot end-effector trajectories in workspace."""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        fig.suptitle('End-Effector Analysis', fontsize=16, fontweight='bold')
        
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        
        for i, (assay_name, df) in enumerate(experiments.items()):
            color = colors[i % len(colors)]
            
            # Compute end-effector trajectory and convert to mm
            trajectory = self.compute_end_effector_trajectory(df) * 1000.0  # Convert from meters to mm
            time_rel = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
            
            # Plot X-Y trajectory
            ax1.plot(trajectory[:, 0], trajectory[:, 1], 
                    label=assay_name, color=color, linewidth=2, alpha=0.8)
            ax1.scatter(trajectory[0, 0], trajectory[0, 1], 
                       color=color, s=100, marker='o', edgecolor='black', 
                       zorder=10, label=f'{assay_name} Start')
            ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], 
                       color=color, s=100, marker='s', edgecolor='black', 
                       zorder=10, label=f'{assay_name} End')
            
            # Plot position vs time
            ax2.plot(time_rel, trajectory[:, 0], 
                    label=f'{assay_name} - X', color=color, linestyle='-')
            ax2.plot(time_rel, trajectory[:, 1], 
                    label=f'{assay_name} - Y', color=color, linestyle='--')
        
        # Check if expected positions are reachable and add warnings
        start_reachable = self.kinematics.is_position_reachable(self.start_position[0]/1000.0, self.start_position[1]/1000.0)
        end_reachable = self.kinematics.is_position_reachable(self.end_position[0]/1000.0, self.end_position[1]/1000.0)
        
        # Add reference positions from experiment configuration
        start_color = 'red' if start_reachable else 'darkred'
        end_color = 'green' if end_reachable else 'darkgreen'
        start_label = 'Expected Start Position' + ('' if start_reachable else ' (UNREACHABLE)')
        end_label = 'Expected End Position' + ('' if end_reachable else ' (UNREACHABLE)')
        
        ax1.scatter(self.start_position[0], self.start_position[1], 
                   color=start_color, s=200, marker='X', edgecolor='black', 
                   zorder=15, label=start_label)
        ax1.scatter(self.end_position[0], self.end_position[1], 
                   color=end_color, s=200, marker='X', edgecolor='black', 
                   zorder=15, label=end_label)
        
        # Add workspace boundary visualization
        self.plot_workspace_boundary(ax1)
        
        # Configure workspace plot
        ax1.set_title('End-Effector Trajectories in Workspace')
        ax1.set_xlabel('X Position (mm)')
        ax1.set_ylabel('Y Position (mm)')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        
        # Configure position vs time plot
        ax2.set_title('End-Effector Position Over Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position (mm)')
        ax2.grid(True, alpha=0.3)
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        
        plt.tight_layout()
        plt.show()
    
    def generate_statistics_report(self, experiments):
        """Generate statistical report of the experiments."""
        print("\n" + "="*80)
        print("EXPERIMENT STATISTICS REPORT")
        print("="*80)
        
        for assay_name, df in experiments.items():
            print(f"\n{assay_name}:")
            print(f"  Duration: {(df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]).total_seconds():.2f} seconds")
            print(f"  Data Points: {len(df)}")
            print(f"  Average Sample Rate: {len(df) / (df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]).total_seconds():.2f} Hz")
            
            # Joint position statistics
            joint_positions = np.array(df['joint_positions'].tolist())
            print(f"  Joint 1 Position Range: {joint_positions[:, 0].min():.3f} to {joint_positions[:, 0].max():.3f} rad")
            print(f"  Joint 2 Position Range: {joint_positions[:, 1].min():.3f} to {joint_positions[:, 1].max():.3f} rad")
            
            # Joint velocity statistics
            joint_velocities = np.array(df['joint_velocities'].tolist())
            print(f"  Max Joint 1 Velocity: {abs(joint_velocities[:, 0]).max():.3f} rad/s")
            print(f"  Max Joint 2 Velocity: {abs(joint_velocities[:, 1]).max():.3f} rad/s")
            
            # End-effector workspace
            trajectory = self.compute_end_effector_trajectory(df)
            workspace_area = (trajectory[:, 0].max() - trajectory[:, 0].min()) * \
                           (trajectory[:, 1].max() - trajectory[:, 1].min())
            print(f"  End-Effector Workspace: {workspace_area*1000000:.2f} mm²")
    
    def save_analysis_plots(self, experiments, output_dir="analysis_output"):
        """Save all analysis plots to files."""
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        # Set matplotlib to non-interactive backend for saving
        plt.ioff()
        
        # Generate timestamp for unique filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save joint analysis
        self.plot_joint_analysis(experiments)
        plt.savefig(f"{output_dir}/joint_analysis_{timestamp}.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        # Save end-effector analysis
        self.plot_end_effector_trajectories(experiments)
        plt.savefig(f"{output_dir}/end_effector_analysis_{timestamp}.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"\nAnalysis plots saved to {output_dir}/")
    
    def run_full_analysis(self, show_plots=True, save_plots=True):
        """Run complete analysis pipeline."""
        print("JiggyJoystick Experiment Analyzer")
        print("="*50)
        print(f"Robot Parameters: L1={self.robot_params['L1']*1000:.1f}mm, L2={self.robot_params['L2']*1000:.1f}mm, d={self.robot_params['d']*1000:.1f}mm")
        
        # Workspace analysis
        self.analyze_workspace()
        
        # Load data
        experiments = self.load_experiment_data()
        if not experiments:
            print("No experiment data found!")
            return
        
        # Generate statistics report
        self.generate_statistics_report(experiments)
        
        if show_plots:
            # Show interactive plots
            self.plot_joint_analysis(experiments)
            self.plot_end_effector_trajectories(experiments)
        
        if save_plots:
            # Save plots to files
            self.save_analysis_plots(experiments)


def main():
    parser = argparse.ArgumentParser(description="Analyze JiggyJoystick experiment logs")
    parser.add_argument("--log-dir", default="/ros2_ws/logs", 
                       help="Directory containing log files")
    parser.add_argument("--no-show", action="store_true", 
                       help="Don't show interactive plots")
    parser.add_argument("--no-save", action="store_true", 
                       help="Don't save plots to files")
    
    args = parser.parse_args()
    
    analyzer = JiggyJoystickAnalyzer(args.log_dir)
    analyzer.run_full_analysis(show_plots=not args.no_show, 
                              save_plots=not args.no_save)


if __name__ == "__main__":
    main()
