# Use an appropriate base image for ROS 2 Jazzy Jalisco
FROM ros:jazzy-ros-base

# Update and install necessary packages
RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    python3-vcstool \
    python3-rosdep \
    cmake \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /ros2_ws

# Copy the package source files
COPY ros2_ws/src /ros2_ws/src

# Install Python dependencies using apt
RUN apt-get update && \
    apt-get install -y python3-numpy python3-yaml && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install dependencies
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# Build the ros workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# Set up microros workspace following original SETUP.md instructions
WORKDIR /microros_ws

# Install additional dependencies for micro-ROS
RUN apt-get update && \
    apt-get install -y \
    flex \
    bison \
    libncurses-dev \
    usbutils \
    libcurl4-openssl-dev \
    clang-tidy \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    libfastcdr-dev \
    libfastrtps-dev \
    && rm -rf /var/lib/apt/lists/*

# Create the Micro-ROS workspace and build the agent
RUN /bin/bash -c "mkdir -p src && \
    cd src && \
    source /opt/ros/jazzy/setup.bash && \
    git clone https://github.com/micro-ROS/micro_ros_setup.git && \
    cd .. && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh"

# Create logs directory
RUN mkdir -p /ros2_ws/logs

# Source ROS 2 setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /microros_ws/install/setup.bash ]; then source /microros_ws/install/setup.bash; fi" >> ~/.bashrc

# Raise the microros agent on the container to wait for the communications from the "uros client" --> Teensy thorugh Serial
RUN echo ""

# Set default command
CMD ["bash"]
