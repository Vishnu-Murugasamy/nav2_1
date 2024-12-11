# Use ROS 2 Humble base image
FROM ros:humble-ros-base

# Set the maintainer label
LABEL maintainer="your-email@example.com"

# Add the Ignition Gazebo repository and install ignition-gazebo6
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y libignition-gazebo6 libignition-gazebo6-dev


# Set the working directory inside the container
WORKDIR /workspace

# Update the package list and install ROS 2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    ament-cmake \
    nano \
    python3-pip \
    python3-vcstool \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-kortex-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-xacro \
    ros-humble-ros-gz-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-rviz2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3 \
    software-properties-common \
    libignition-transport11-dev \
    libgflags-dev \
    ros-humble-kinematics-interface-kdl && \
    rm -rf /var/lib/apt/lists/*



# Set up ROS 2 workspace
RUN mkdir -p /workspace/src

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint to our custom script
ENTRYPOINT ["/entrypoint.sh"]

# Set the default command
CMD ["bash"]

