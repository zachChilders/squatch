FROM osrf/ros:humble-desktop-full

# Install additional development tools and UART utilities
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgeographic-dev \
    geographiclib-tools \
    nlohmann-json3-dev \
    nano \
    vim \
    gdb \
    strace \
    usbutils \
    screen \
    minicom \
    socat \
    picocom \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user with device access groups
RUN useradd -m -s /bin/bash -G dialout,plugdev,tty ros_dev
RUN echo 'ros_dev ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Create workspace
USER ros_dev
WORKDIR /home/ros_dev
RUN mkdir -p ros2_ws/src

# Source ROS2 in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc

# Set up workspace
WORKDIR /home/ros_dev/ros2_ws

# Initialize rosdep
USER root
RUN rosdep update
USER ros_dev

CMD ["/bin/bash"]