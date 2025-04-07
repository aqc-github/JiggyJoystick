To develop the ROS 2 nodes when using the dev container follow these steps:
(currently only supporting Apple SiliconMacOS)

1. Clone the repository:
   ```
   git clone https://github.com/JiggyJoystick/ros2_ws.git
   ```
2. Navigate to the cloned repository:
   ```
   cd ros2_ws
   ```
3. Install the VS Code extension for Remote Development. In the VS Code Extensions view, search for "Remote Development" and install the extension.

4. When opening the repo in the command pallet (Shift+Command+P) select "Remote-Containers: Open Folder in Container"

5. Go grab a coffee, this will take a while.

6. Once the container is ready, you can start developing your ROS 2 nodes.

7. Once the container is ready try to run the following commands to test the setup:

  - Source the ROS2 environment:
    ```
    source /opt/ros/jazzy/setup.bash
    ```
  - Build the workspace:
    ```
    colcon build
    ```
  - In separate terminals run the following commands:
    ```
    // In the first terminal
    source install/setup.bash
    ros2 run my_pubsub_pkg subscriber
    // IN the second terminals
    source install/setup.bash
    ros2 run demo_nodes_cpp publisher
    ```

If everything is working correctly, you should see the subscriber receiving messages from the publisher. And everything being logged to the console and to a log file.
