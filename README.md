# **Nav2_1: ROS 2 Navigation with Docker**

This repository provides an easy-to-follow guide for setting up and running the **Nav2** stack for TurtleBot3 simulation using Docker and **ROS 2 Humble**.

## **Prerequisites**
Before starting, make sure you have Docker installed on your system.

- [Install Docker](https://docs.docker.com/get-docker/)
- [Install ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)

## **Setup Instructions**

1. **Clone the repository**:
   Clone the repository to your local machine using the following command:
   ```bash
   git clone https://github.com/Vishnu-Murugasamy/nav2_1.git

2. **Build the Docker Image**:
   Navigate into the project folder and build the Docker image:
   ```bash
   cd nav2_1
   docker build -t nav2 .
3. **Allow Docker to access your display**:
   Run the following command to allow Docker containers to access your X11 display:
   ```bash
   xhost +local:docker
4. **Run the Docker Container**:
   Start the Docker container with the following command. This will mount the X11 socket for GUI support and run the container interactively:
   ```bash
   docker run -it --rm \
     --privileged \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -e DISPLAY=$DISPLAY \
     --name nav2_1 \
     nav2
5. **Source ROS 2 and Set TurtleBot3 Model**:
   Inside the running Docker container, source the ROS 2 environment and set the TurtleBot3 model (only required if you are using Gazebo Classic):
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models  # Iron and older only with Gazebo Classic
6. **Modify the Gazebo World File**:

To modify the `gazebo_launch_world.world` file, you can update the `<world>` element as follows:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <!-- Add ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Scene settings -->
    <scene>
      <shadows>false</shadows>
    </scene>

    <!-- GUI settings -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <!-- Physics settings -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Add TurtleBot3 Waffle -->
    <include>
      <uri>model://turtlebot3_waffle</uri>
      <pose>1.0 2.0 0.0 0 0 0</pose> <!-- x, y, z, roll, pitch, yaw -->
    </include>

  </world>
</sdf>

 
7. **Launch the TurtleBot3 Simulation**:
   Now, you can launch the simulation using ROS 2. Run the following command to start the TurtleBot3 simulation in Gazebo:
   ```bash
   ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False



