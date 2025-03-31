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