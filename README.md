# A-HAT-TRIC
Autonomous Hybrid Aerial-Terrestrial Transport and Retrieval for In-situ Collection

## Pre-Requisites
- Ubuntu 22.04
- ROS2 Humble
- QGroundControl

## Instructions for Launching Simulation

## Instructions for Deploying on Actual Vehicle

## Setting up Simulation Environment
1. Navigate to the directory you'd like to work in.
2. Clone the `PX4-Autopilot` repository to your local machine.
```
   git clone https://github.com/PX4/PX4-Autopilot.git -b release/1.14 --recursive
```
3. Install necessary prerequisites for this toolkit.
```
   ./Tools/setup/ubuntu.sh && pip uninstall em && pip install empy==3.3.4
```
4. In the `PX4-Autopilot` folder, run the command below. If a window called "Gazebo Sim" appears, this toolkit is functioning.
```
   make px4_sitl gz_x500_depth
```
5. At the same level as the `PX4-Autopilot` folder, create a folder that will allow ROS2 to use Gazebo Garden.
```
   mkdir -p ros2_gzbridge/src
```
6. Now clone the repository containing the tools necessary for this interface.
```
   cd ros2_gzbridge/src && git clone https://github.com/gazebosim/ros_gz.git -b humble
```
7. Assign the proper Gazebo version.
```
   export GZ_VERSION=garden
```
8. Install the necessary prerequisites.
```
 cd .. && rosdep install -r --from-paths src -i -y --rosdistro humble
```
9. Source your ROS2 Humble installation, `source /opt/ros/humble/setup.bash`, and build the workspace.
```
   colcon build
```
NOTE: It is recommended to add the ROS2 source command to the `.bashrc` script that runs at each instance of a new terminal being opened.

10. At the level of the `PX4-Autopilot` and `ros2_gzbridge` folders, create a 'models' directory.
```
   mkdir models
```
11. Copy the script `set_GZ_SIM_RESOURCE_PATH.sh` from this repository (in the `helper_scripts` directory) to your `PX4-Autopilot` folder. Be sure to update the path to the `models` folder to agree with your machine. The `pwd` command will be helpful for this.
