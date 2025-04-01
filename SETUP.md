# Set up

## Installing ROS 2 Jazzy Jalisco LTS on Ubuntu 24.04 LTS

### 1. Set up the locale:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
Important to have a _locale_ that supports UTF-8. technically Ubuntu should support this but better safe than sorry.

### 2. Set up the sources:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

###  3. Install development tools and ROS version:

```bash
sudo apt install ros-dev-tools
```

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-ros-base
```
We are going to be working on a headless setup with remote connecttion to the raspberry through ssh, so ROS2 barebones is good for now.
And also it is more lightweitgh for our RPi5. But to test that the installation works correctly, the desktop version comes with some very good examples.

### Opt. 4. Check the ROS2 setup:
If instead of the barebones, you want ot check that the installation works correctly install the _desktop_ version.

```bash
sudo apt install ros-jazzy-desktop
```

And the in two different terminals run the talker and listener demo nodes:

Terminal 1 ->
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

Terminal 2 ->
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

In both terminals you should see a publisher and a subscriber that send and receive the information. The _talker_ runs in a cpp node and the _listener_ in a python3 node, which also helps check that both languages work and have their respective installations properly executed.

## Installing Microros:
Since part of our application will be running on the Raspberry Pi, we need to install the Micro-ROS agent to enable communication between ROS2 and the Raspberry Pi.

### 5. Install Micro-ROS Agent:
To install the Micro-ROS agent, follow these steps:

0. Create the Micro-ROS workspace:
```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
```

1. Clone the Micro-ROS repository:
```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/micro-ROS/micro_ros_setup.git
```

2. Navigate to the cloned repository:
```bash
cd micro_ros_setup
```

3. Update dependencies using rosdep:
```bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install python3-pip
```

4. Build Micro-ROS tools and source them:
```bash
colcon build
source install/local_setup.bash
```

4. Install Micro-ROS Agent:
```bash
ros2 run micro_ros_setup create_agent_ws.sh
```

5. Build the Agent and source it:
```bash
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

6. Verify the installation by _dry running_ the agent:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

If the installation is successful, you should see in the terminal the following message or similar:
> [1743496716.924499] info     | TermiosAgentLinux.cpp | init                     | Serial port not found. | device: /dev/ttyACM0, error 2, waiting for connection...

The message displays the status of the serial port connection, and since we have not build the device yet, it shall not work.
