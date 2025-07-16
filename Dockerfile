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

# Create logs directory
RUN mkdir -p /ros2_ws/logs

# Source ROS 2 setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Go back to ROS2 workspace
WORKDIR /ros2_ws

# Set up the entrypoint
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV PYTHONPATH=/ros2_ws/install/lib/python3.12/site-packages:$PYTHONPATH

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
