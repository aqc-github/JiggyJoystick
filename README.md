# Dynamic Mouse Joystick

# ROS2 Package
## Running ROS2 Publisher and Subscriber Nodes

```bash
source ~/[workspace_path]/install/setup.bash
```

After sourcing your package, you can run your publisher and subscriber nodes using the `ros2 run` command.

## Run the Publisher Node

```bash
ros2 run my_pubsub_pkg publisher
```

## Run the Subscriber Node
In a new terminal (after sourcing your workspace and ROS2 setup files):

```bash
ros2 run my_pubsub_pkg subscriber
```

## Important Notes:

1. Make sure both terminal windows have sourced:
   - ROS2 installation: `source /opt/ros/[distro]/setup.bash`
   - Your workspace: `source ~/[workspace_path]/install/setup.bash`

2. The exact node names depend on how they are defined in your `setup.py` or CMakeLists.txt file. If your node names are different, you'll need to use those names instead of "publisher" and "subscriber".

3. You can verify available nodes in your package with:
   ```bash
   ros2 pkg executables my_pubsub_pkg
   ```
