#!/usr/bin/env python3
"""
Test script to demonstrate the complete workflow:
1. Run a simulated experiment (using existing logs)
2. Execute the post-experiment analysis
3. Display the results

This simulates what happens after a real experiment completes.
"""

import subprocess
import sys
import os
import time
from datetime import datetime

def run_experiment_analysis():
    """Run the experiment analyzer on existing log data"""
    print("🧪 JiggyJoystick Post-Experiment Analysis Test")
    print("=" * 60)
    
    # Check if we're in the Docker container
    container_env = os.path.exists('/ros2_ws')
    
    if container_env:
        # Running inside Docker container
        analyzer_path = '/ros2_ws/experiment_analyzer.py'
        logs_path = '/ros2_ws/logs'
    else:
        # Running on host - would need to adjust paths
        print("❌ This test should be run inside the Docker container")
        return False
    
    print(f"📊 Analyzing logs from: {logs_path}")
    print(f"🔧 Using analyzer: {analyzer_path}")
    
    # Check if analyzer exists
    if not os.path.exists(analyzer_path):
        print(f"❌ Analyzer script not found: {analyzer_path}")
        return False
    
    # Check if logs exist
    if not os.path.exists(logs_path):
        print(f"❌ Logs directory not found: {logs_path}")
        return False
    
    # Count available log files
    log_files = [f for f in os.listdir(logs_path) if f.endswith('.csv')]
    print(f"📁 Found {len(log_files)} log files to analyze")
    
    if len(log_files) == 0:
        print("⚠️  No log files found. Running a quick experiment simulation...")
        # This would normally run an actual experiment
        print("   (In a real scenario, an experiment would have been completed)")
        return False
    
    # Run the analyzer
    print("\n🚀 Starting analysis...")
    start_time = time.time()
    
    try:
        result = subprocess.run([
            sys.executable, analyzer_path, '--no-show'
        ], capture_output=True, text=True, timeout=120)
        
        analysis_time = time.time() - start_time
        
        if result.returncode == 0:
            print(f"✅ Analysis completed successfully in {analysis_time:.2f} seconds!")
            print("📈 Generated comprehensive analysis including:")
            print("   - Joint position, velocity, and effort analysis")
            print("   - End-effector trajectory reconstruction")
            print("   - Statistical summary of robot performance")
            print("   - High-resolution plots saved to analysis_output/")
            
            # Show some key results from the output
            if result.stdout:
                lines = result.stdout.split('\n')
                print("\n📋 Analysis Summary:")
                print("-" * 40)
                
                # Look for key statistics
                in_stats = False
                for line in lines:
                    if "EXPERIMENT STATISTICS REPORT" in line:
                        in_stats = True
                        continue
                    elif in_stats and line.strip():
                        if line.startswith("Assay"):
                            print(f"🧪 {line}")
                        elif "Duration:" in line or "Data Points:" in line or "Sample Rate:" in line:
                            print(f"   {line}")
                        elif "Position Range:" in line or "Velocity:" in line:
                            print(f"   {line}")
                        elif "Workspace:" in line:
                            print(f"   {line}")
            
            # Check if plots were saved
            output_dir = 'analysis_output'
            if os.path.exists(output_dir):
                plot_files = [f for f in os.listdir(output_dir) if f.endswith('.png')]
                print(f"\n📊 Generated {len(plot_files)} visualization files:")
                for plot_file in plot_files:
                    print(f"   - {plot_file}")
            
            return True
            
        else:
            print(f"❌ Analysis failed with exit code {result.returncode}")
            if result.stderr:
                print(f"Error details: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("⏰ Analysis timed out after 2 minutes")
        return False
    except Exception as e:
        print(f"❌ Failed to run analysis: {e}")
        return False

def main():
    """Main test function"""
    print(f"🕐 Test started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    success = run_experiment_analysis()
    
    if success:
        print("\n🎉 Post-experiment analysis workflow test PASSED!")
        print("🔄 This analysis would automatically run after each real experiment")
        print("📧 In a production setup, results could be emailed or uploaded to a dashboard")
    else:
        print("\n❌ Post-experiment analysis workflow test FAILED!")
        print("🔧 Check the error messages above for troubleshooting")
    
    print(f"\n🕐 Test completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
