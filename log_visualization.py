import pandas as pd
import matplotlib.pyplot as plt
def plot_log_file(filename):
    # Read the CSV log file
    data = pd.read_csv(filename)

    # Convert string representation of lists to actual lists
    data['joint_positions'] = data['joint_positions'].apply(eval)
    data['joint_velocities'] = data['joint_velocities'].apply(eval)
    data['joint_efforts'] = data['joint_efforts'].apply(eval)

    # Extract data
    timestamps = pd.to_datetime(data['timestamp'])
    joint_positions = pd.DataFrame(data['joint_positions'].to_list(), index=timestamps)
    joint_velocities = pd.DataFrame(data['joint_velocities'].to_list(), index=timestamps)
    joint_efforts = pd.DataFrame(data['joint_efforts'].to_list(), index=timestamps)

    # Plot joint positions
    plt.figure(figsize=(12, 6))
    for column in joint_positions.columns:
        plt.plot(joint_positions.index, joint_positions[column], label=f'Joint Position {column + 1}')
    plt.title('Joint Positions Over Time')
    plt.xlabel('Time')
    plt.ylabel('Position (radians)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot joint velocities
    plt.figure(figsize=(12, 6))
    for column in joint_velocities.columns:
        plt.plot(joint_velocities.index, joint_velocities[column], label=f'Joint Velocity {column + 1}')
    plt.title('Joint Velocities Over Time')
    plt.xlabel('Time')
    plt.ylabel('Velocity (radians/s)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot joint efforts
    plt.figure(figsize=(12, 6))
    for column in joint_efforts.columns:
        plt.plot(joint_efforts.index, joint_efforts[column], label=f'Joint Effort {column + 1}')
    plt.title('Joint Efforts Over Time')
    plt.xlabel('Time')
    plt.ylabel('Effort (Nm)')
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage
plot_log_file('/ros2_ws/logs/17-04-03-25-07-assay1.csv')
