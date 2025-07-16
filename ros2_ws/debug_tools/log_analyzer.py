#!/usr/bin/env python3
"""
JiggyJoystick Log Analyzer
Tool for analyzing experiment logs and generating insights
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import argparse
import json
import os
import re
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

class LogAnalyzer:
    """
    Analyze JiggyJoystick experiment logs
    """
    
    def __init__(self, log_dir='./logs'):
        self.log_dir = Path(log_dir)
        self.log_files = []
        self.experiment_data = {}
        self.summary_stats = {}
        
    def discover_logs(self):
        """Discover all log files in the log directory"""
        if not self.log_dir.exists():
            print(f"Log directory {self.log_dir} does not exist")
            return
            
        # Find CSV log files
        csv_files = list(self.log_dir.glob('log_assay*.csv'))
        
        # Find JSON debug files
        json_files = list(self.log_dir.glob('topic_debug*.json'))
        
        self.log_files = {
            'csv': csv_files,
            'json': json_files
        }
        
        print(f"Found {len(csv_files)} CSV log files and {len(json_files)} JSON debug files")
        
    def parse_log_filename(self, filename):
        """Parse information from log filename"""
        match = re.match(r'log_assay(\d+)_trial(\d+)_(\d{8}_\d{6})\.csv', filename.name)
        if match:
            return {
                'assay': int(match.group(1)),
                'trial': int(match.group(2)),
                'timestamp': match.group(3)
            }
        return None
    
    def load_csv_logs(self):
        """Load all CSV log files"""
        if not self.log_files['csv']:
            print("No CSV log files found")
            return
            
        all_data = []
        
        for csv_file in self.log_files['csv']:
            file_info = self.parse_log_filename(csv_file)
            if not file_info:
                continue
                
            try:
                df = pd.read_csv(csv_file)
                df['assay'] = file_info['assay']
                df['trial'] = file_info['trial']
                df['log_timestamp'] = file_info['timestamp']
                
                # Convert timestamp to datetime
                df['timestamp'] = pd.to_datetime(df['timestamp'])
                
                # Parse joint positions and velocities if they're strings
                if 'joint_positions' in df.columns:
                    df['joint_positions'] = df['joint_positions'].apply(self.parse_array_string)
                if 'joint_velocities' in df.columns:
                    df['joint_velocities'] = df['joint_velocities'].apply(self.parse_array_string)
                if 'torques' in df.columns:
                    df['torques'] = df['torques'].apply(self.parse_array_string)
                
                all_data.append(df)
                
            except Exception as e:
                print(f"Error loading {csv_file}: {e}")
                continue
        
        if all_data:
            self.experiment_data = pd.concat(all_data, ignore_index=True)
            self.experiment_data = self.experiment_data.sort_values(['assay', 'trial', 'timestamp'])
            print(f"Loaded {len(self.experiment_data)} data points from {len(all_data)} trials")
        else:
            print("No valid data loaded")
    
    def parse_array_string(self, array_str):
        """Parse array string from CSV"""
        if pd.isna(array_str) or array_str == '':
            return []
        
        # Try to parse as list representation
        try:
            # Remove brackets and split by comma
            cleaned = str(array_str).strip('[]')
            if cleaned:
                return [float(x.strip()) for x in cleaned.split(',')]
            return []
        except:
            return []
    
    def generate_summary_stats(self):
        """Generate summary statistics for the experiment"""
        if self.experiment_data.empty:
            return
            
        summary = {}
        
        # Basic experiment info
        summary['total_trials'] = len(self.experiment_data[['assay', 'trial']].drop_duplicates())
        summary['total_assays'] = self.experiment_data['assay'].nunique()
        summary['total_data_points'] = len(self.experiment_data)
        
        # Trial durations
        trial_durations = []
        for (assay, trial), group in self.experiment_data.groupby(['assay', 'trial']):
            if len(group) > 1:
                duration = (group['timestamp'].max() - group['timestamp'].min()).total_seconds()
                trial_durations.append(duration)
        
        if trial_durations:
            summary['avg_trial_duration'] = np.mean(trial_durations)
            summary['min_trial_duration'] = np.min(trial_durations)
            summary['max_trial_duration'] = np.max(trial_durations)
        
        # Trial success rates
        if 'trial_success' in self.experiment_data.columns:
            trial_success = self.experiment_data.groupby(['assay', 'trial'])['trial_success'].last()
            summary['success_rate'] = trial_success.mean() if len(trial_success) > 0 else 0
            summary['successful_trials'] = trial_success.sum() if len(trial_success) > 0 else 0
        
        # Joint position statistics
        joint_positions = []
        for _, row in self.experiment_data.iterrows():
            if row['joint_positions'] and len(row['joint_positions']) >= 2:
                joint_positions.append(row['joint_positions'])
        
        if joint_positions:
            joint_positions = np.array(joint_positions)
            summary['joint1_mean'] = np.mean(joint_positions[:, 0])
            summary['joint1_std'] = np.std(joint_positions[:, 0])
            summary['joint2_mean'] = np.mean(joint_positions[:, 1])
            summary['joint2_std'] = np.std(joint_positions[:, 1])
            summary['joint1_range'] = np.ptp(joint_positions[:, 0])
            summary['joint2_range'] = np.ptp(joint_positions[:, 1])
        
        # Data quality metrics
        summary['data_completeness'] = {}
        for col in ['joint_positions', 'joint_velocities', 'torques']:
            if col in self.experiment_data.columns:
                non_empty = self.experiment_data[col].apply(lambda x: len(x) > 0 if isinstance(x, list) else False)
                summary['data_completeness'][col] = non_empty.mean()
        
        self.summary_stats = summary
        
    def print_summary_report(self):
        """Print comprehensive summary report"""
        print("=" * 80)
        print("📊 JIGGYJOYSTICK EXPERIMENT ANALYSIS REPORT")
        print("=" * 80)
        print(f"Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print()
        
        if not self.summary_stats:
            print("No summary statistics available")
            return
            
        # Basic info
        print("🔬 EXPERIMENT OVERVIEW")
        print("-" * 40)
        print(f"Total Assays: {self.summary_stats.get('total_assays', 'N/A')}")
        print(f"Total Trials: {self.summary_stats.get('total_trials', 'N/A')}")
        print(f"Total Data Points: {self.summary_stats.get('total_data_points', 'N/A')}")
        print()
        
        # Trial performance
        print("⏱️  TRIAL PERFORMANCE")
        print("-" * 40)
        if 'avg_trial_duration' in self.summary_stats:
            print(f"Average Trial Duration: {self.summary_stats['avg_trial_duration']:.2f} seconds")
            print(f"Min Trial Duration: {self.summary_stats['min_trial_duration']:.2f} seconds")
            print(f"Max Trial Duration: {self.summary_stats['max_trial_duration']:.2f} seconds")
        
        if 'success_rate' in self.summary_stats:
            print(f"Success Rate: {self.summary_stats['success_rate']:.2%}")
            print(f"Successful Trials: {self.summary_stats['successful_trials']}")
        print()
        
        # Joint statistics
        print("🤖 JOINT MOTION STATISTICS")
        print("-" * 40)
        if 'joint1_mean' in self.summary_stats:
            print(f"Joint 1 - Mean: {self.summary_stats['joint1_mean']:.4f} rad, "
                  f"Std: {self.summary_stats['joint1_std']:.4f} rad, "
                  f"Range: {self.summary_stats['joint1_range']:.4f} rad")
            print(f"Joint 2 - Mean: {self.summary_stats['joint2_mean']:.4f} rad, "
                  f"Std: {self.summary_stats['joint2_std']:.4f} rad, "
                  f"Range: {self.summary_stats['joint2_range']:.4f} rad")
        print()
        
        # Data quality
        print("📈 DATA QUALITY")
        print("-" * 40)
        if 'data_completeness' in self.summary_stats:
            for metric, value in self.summary_stats['data_completeness'].items():
                print(f"{metric}: {value:.2%} complete")
        print()
    
    def plot_joint_trajectories(self, assay_num=None, trial_num=None, save_plot=False):
        """Plot joint trajectories for specific trials or all trials"""
        if self.experiment_data.empty:
            print("No data to plot")
            return
            
        # Filter data
        data = self.experiment_data.copy()
        if assay_num is not None:
            data = data[data['assay'] == assay_num]
        if trial_num is not None:
            data = data[data['trial'] == trial_num]
        
        if data.empty:
            print("No data matches the specified criteria")
            return
        
        # Extract joint positions
        joint_data = []
        for _, row in data.iterrows():
            if row['joint_positions'] and len(row['joint_positions']) >= 2:
                joint_data.append({
                    'timestamp': row['timestamp'],
                    'assay': row['assay'],
                    'trial': row['trial'],
                    'joint1': row['joint_positions'][0],
                    'joint2': row['joint_positions'][1]
                })
        
        if not joint_data:
            print("No valid joint position data found")
            return
        
        joint_df = pd.DataFrame(joint_data)
        
        # Create plots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Joint positions over time
        for (assay, trial), group in joint_df.groupby(['assay', 'trial']):
            group = group.sort_values('timestamp')
            time_rel = (group['timestamp'] - group['timestamp'].min()).dt.total_seconds()
            
            axes[0, 0].plot(time_rel, group['joint1'], label=f'A{assay}T{trial}', alpha=0.7)
            axes[0, 1].plot(time_rel, group['joint2'], label=f'A{assay}T{trial}', alpha=0.7)
        
        axes[0, 0].set_title('Joint 1 Position Over Time')
        axes[0, 0].set_xlabel('Time (seconds)')
        axes[0, 0].set_ylabel('Position (radians)')
        axes[0, 0].grid(True)
        axes[0, 0].legend()
        
        axes[0, 1].set_title('Joint 2 Position Over Time')
        axes[0, 1].set_xlabel('Time (seconds)')
        axes[0, 1].set_ylabel('Position (radians)')
        axes[0, 1].grid(True)
        axes[0, 1].legend()
        
        # Plot 2: Joint space trajectory
        axes[1, 0].scatter(joint_df['joint1'], joint_df['joint2'], 
                          c=joint_df['assay'], cmap='viridis', alpha=0.6)
        axes[1, 0].set_title('Joint Space Trajectory')
        axes[1, 0].set_xlabel('Joint 1 Position (radians)')
        axes[1, 0].set_ylabel('Joint 2 Position (radians)')
        axes[1, 0].grid(True)
        
        # Plot 3: Position distributions
        axes[1, 1].hist(joint_df['joint1'], bins=30, alpha=0.7, label='Joint 1')
        axes[1, 1].hist(joint_df['joint2'], bins=30, alpha=0.7, label='Joint 2')
        axes[1, 1].set_title('Joint Position Distributions')
        axes[1, 1].set_xlabel('Position (radians)')
        axes[1, 1].set_ylabel('Frequency')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'joint_trajectories_{timestamp}.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Saved joint trajectories plot to {filename}")
        
        plt.show()
    
    def plot_trial_success_analysis(self, save_plot=False):
        """Plot trial success analysis"""
        if self.experiment_data.empty or 'trial_success' not in self.experiment_data.columns:
            print("No trial success data available")
            return
            
        # Get success data per trial
        success_data = self.experiment_data.groupby(['assay', 'trial']).agg({
            'trial_success': 'last'
        }).reset_index()
        
        # Create plots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Success rate by assay
        success_by_assay = success_data.groupby('assay')['trial_success'].agg(['mean', 'count']).reset_index()
        axes[0, 0].bar(success_by_assay['assay'], success_by_assay['mean'])
        axes[0, 0].set_title('Success Rate by Assay')
        axes[0, 0].set_xlabel('Assay Number')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Plot 2: Success rate by trial within assay
        for assay in success_data['assay'].unique():
            assay_data = success_data[success_data['assay'] == assay]
            success_by_trial = assay_data.groupby('trial')['trial_success'].mean()
            axes[0, 1].plot(success_by_trial.index, success_by_trial.values, 
                           marker='o', label=f'Assay {assay}')
        
        axes[0, 1].set_title('Success Rate by Trial Number')
        axes[0, 1].set_xlabel('Trial Number')
        axes[0, 1].set_ylabel('Success Rate')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: Overall success distribution
        success_counts = success_data['trial_success'].value_counts()
        axes[1, 0].pie(success_counts.values, labels=['Failed', 'Success'], 
                       autopct='%1.1f%%', startangle=90)
        axes[1, 0].set_title('Overall Trial Success Distribution')
        
        # Plot 4: Success timeline
        success_data_sorted = success_data.sort_values(['assay', 'trial'])
        success_data_sorted['trial_index'] = range(len(success_data_sorted))
        
        colors = ['red' if not success else 'green' for success in success_data_sorted['trial_success']]
        axes[1, 1].scatter(success_data_sorted['trial_index'], 
                          success_data_sorted['trial_success'], 
                          c=colors, alpha=0.6)
        axes[1, 1].set_title('Success Timeline')
        axes[1, 1].set_xlabel('Trial Index')
        axes[1, 1].set_ylabel('Success (1) / Failure (0)')
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'trial_success_analysis_{timestamp}.png'
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Saved trial success analysis to {filename}")
        
        plt.show()
    
    def export_analysis_report(self, filename=None):
        """Export comprehensive analysis report"""
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'experiment_analysis_{timestamp}.json'
        
        report = {
            'generation_time': datetime.now().isoformat(),
            'log_directory': str(self.log_dir),
            'files_analyzed': {
                'csv_files': [str(f) for f in self.log_files['csv']],
                'json_files': [str(f) for f in self.log_files['json']]
            },
            'summary_statistics': self.summary_stats,
            'data_shape': {
                'rows': len(self.experiment_data),
                'columns': list(self.experiment_data.columns) if not self.experiment_data.empty else []
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        print(f"Analysis report exported to {filename}")
        return filename

def main():
    parser = argparse.ArgumentParser(description='JiggyJoystick Log Analyzer')
    parser.add_argument('--log-dir', default='./logs', 
                       help='Directory containing log files')
    parser.add_argument('--plot-trajectories', action='store_true',
                       help='Plot joint trajectories')
    parser.add_argument('--plot-success', action='store_true',
                       help='Plot trial success analysis')
    parser.add_argument('--assay', type=int,
                       help='Specific assay to analyze')
    parser.add_argument('--trial', type=int,
                       help='Specific trial to analyze')
    parser.add_argument('--save-plots', action='store_true',
                       help='Save plots to files')
    parser.add_argument('--export-report', action='store_true',
                       help='Export analysis report to JSON')
    
    args = parser.parse_args()
    
    # Initialize analyzer
    analyzer = LogAnalyzer(args.log_dir)
    
    # Discover and load logs
    analyzer.discover_logs()
    analyzer.load_csv_logs()
    
    if analyzer.experiment_data.empty:
        print("No experiment data loaded. Check log directory and file formats.")
        return
    
    # Generate summary
    analyzer.generate_summary_stats()
    analyzer.print_summary_report()
    
    # Generate plots
    if args.plot_trajectories:
        analyzer.plot_joint_trajectories(
            assay_num=args.assay,
            trial_num=args.trial,
            save_plot=args.save_plots
        )
    
    if args.plot_success:
        analyzer.plot_trial_success_analysis(save_plot=args.save_plots)
    
    # Export report
    if args.export_report:
        analyzer.export_analysis_report()

if __name__ == '__main__':
    main()
