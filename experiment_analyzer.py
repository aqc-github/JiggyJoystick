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


class JiggyJoystickAnalyzer:
    def __init__(self, log_directory="/ros2_ws/logs"):
        self.log_directory = log_directory
        self.robot_params = {
            'L1': 0.0635,  # Link 1 length (m)
            'L2': 0.0635,  # Link 2 length (m)
            'd': 0.0508,   # Base width (m)
        }
        
    def forward_kinematics(self, theta1, theta2):
        """
        Compute end-effector position from joint angles.
        Returns (x, y) position of end-effector.
        """
        L1, L2, d = self.robot_params['L1'], self.robot_params['L2'], self.robot_params['d']
        
        # Position of joint 1 end (left arm)
        x1 = L1 * np.cos(theta1)
        y1 = L1 * np.sin(theta1)
        
        # Position of joint 2 end (right arm, offset by base width)
        x2 = d + L2 * np.cos(theta2)
        y2 = L2 * np.sin(theta2)
        
        # End-effector is at the intersection of the two circles
        # For simplicity, we'll use the midpoint approximation
        # In a real implementation, you'd solve the circle intersection
        x_ee = (x1 + x2) / 2
        y_ee = (y1 + y2) / 2
        
        return x_ee, y_ee
    
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
            
            # Compute end-effector trajectory
            trajectory = self.compute_end_effector_trajectory(df)
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
        
        # Configure workspace plot
        ax1.set_title('End-Effector Trajectories in Workspace')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        
        # Configure position vs time plot
        ax2.set_title('End-Effector Position Over Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position (m)')
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
