Nav2_1: ROS 2 Navigation with Docker
This repository contains instructions to run the Nav2 stack using Docker for simulating a TurtleBot3 with ROS 2 Humble and Gazebo.

Prerequisites
Before running the commands, make sure you have Docker and ROS 2 Humble installed on your system.

Install Docker
Install ROS 2 Humble
Setup Instructions
Clone the repository: Clone the repository to your local machine using the following command:

bash
Copy code
git clone https://github.com/Vishnu-Murugasamy/nav2_1.git
Build the Docker Image: Navigate into the project folder and build the Docker image:

bash
Copy code
cd nav2_1
docker build -t nav2 .
Allow Docker to access your display: Run the following command to allow Docker containers to access your X11 display:

bash
Copy code
xhost +local:docker
Run the Docker Container: Start the Docker container with the following command. This will mount the X11 socket for GUI support and run the container interactively:

bash
Copy code
docker run -it --rm \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --name nav2_1 \
  nav2
Source ROS 2 and Set TurtleBot3 Model: Inside the running Docker container, source the ROS 2 environment and set the TurtleBot3 model (only required if you are using Gazebo Classic):

bash
Copy code
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models  # Iron and older only with Gazebo Classic
Launch the TurtleBot3 Simulation: Now, you can launch the simulation using ROS 2. Run the following command to start the TurtleBot3 simulation in Gazebo:

bash
Copy code
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
Troubleshooting
If you encounter any issues with GUI or display settings, make sure the xhost command was run properly, and Docker has access to the X11 display.
If the Gazebo simulation doesn't launch, verify that you have the correct versions of ROS 2 and Docker installed.
Additional Resources
ROS 2 Documentation
TurtleBot3 Documentation
