# A-HAT-TRIC
Autonomous Hybrid Aerial-Terrestrial Transport and Retrieval for In-situ Collection
### Collaborators
Madelyne Rossmann, Mahima Thirukkonda, Sabrina Zaleski
### Acknowledgements
Dr. Derek Paley, Dr. Joe Conroy
### Project Overview
Inspired by the canceled NASA JPL 

## Pre-Requisites
- Ubuntu 22.04
- [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

## Instructions for Launching Simulation

## Instructions for Deploying on Actual Vehicle

## Setting up Simulation Environment
Computer simulation is primarily leveraged in this project for testing code that does not require camera information. Camera information is difficult to accurately replicate in simulation, so the general command algorithm is tested in simulation before vision information is integrated for deployment on the actual vehicle. At this stage of the development, only the aerial configuration is set up to be tested in simulation.
1. Navigate to the directory you'd like to work in.
2. Clone the `PX4-Autopilot` repository to your local machine.
```
   git clone https://github.com/PX4/PX4-Autopilot.git -b release/1.14 --recursive
```
3. Install necessary prerequisites for this toolkit.
```
   ./Tools/setup/ubuntu.sh
   pip uninstall em && pip install empy==3.3.4
```
4. In the `PX4-Autopilot` folder, run the command below. If a window called "Gazebo Sim" appears, this toolkit is functioning.
```
   make px4_sitl gz_x500_depth
```
5. Download ROS2 Humble and install necessary prerequisities. Use the script `install_ros2h_notsource_root.sh` in the `helper_scripts` directory to do so.
```
   sudo chmod +x install_ros2h_notsource_root.sh
   sudo ./install_ros2h_notsource_root.sh
```
Source your ROS2 Humble installation, `source /opt/ros/humble/setup.bash`. 

NOTE: It is recommended to add the ROS2 source command to the `.bashrc` script that runs at each instance of a new terminal being opened.

6. At the same level as the `PX4-Autopilot` folder, create a folder that will allow ROS2 to use Gazebo Garden.
```
   mkdir -p ros2_gzbridge/src
```
7. Now clone the repository containing the tools necessary for this interface.
```
   cd ros2_gzbridge/src
   git clone https://github.com/gazebosim/ros_gz.git -b humble
```
8. Assign the proper Gazebo version.
```
   export GZ_VERSION=garden
```
9. Install the necessary prerequisites.
```
 cd ..
 rosdep install -r --from-paths src -i -y --rosdistro humble
```
10. Build the workspace.
```
   colcon build
```
11. At the level of the `PX4-Autopilot` and `ros2_gzbridge` folders, create a 'models' directory.
```
   mkdir models
```
12. Copy the script `set_GZ_SIM_RESOURCE_PATH.sh` from this repository (in the `helper_scripts` directory) to your `PX4-Autopilot` folder. Be sure to update the path to the `models` folder on your machine. The `pwd` command will be helpful for this.
13. Copy the file `baylands.sdf` from this repository (in the `helper_scripts` directory) to `PX4-Autopilot/Tools/simulation/gz/worlds/`.
14. Copy the script `run_baylands_total.sh` from this repository (in the `helper_scripts` directory) to your `PX4-Autopilot` folder. Be sure the file path in line 6 agrees with your local machine. Again, `pwd` will be helpful here.
15. In the `PX4-Autopilot` folder, run the below commands. Upon startup, you should see the vehicle in a park-like setting. At this point, QGroundControl can be used to manually fly the vehicle (be sure Virtual Joystick is enabled in Application Settings).
```
   ./set_GZ_SIM_RESOURCE_PATH.sh
   ./run_baylands_total.sh
```
16. Another toolkit must be installed to allow the simulation to access Mavlink ROS2 messages. Install this toolkit in the root directory, where `PX4-Autopilot` and `ros2_gzbridge` are located, following the [instructions](https://docs.px4.io/main/en/middleware/uxrce_dds.html) provided by PX4.
17. Copy the script `start_uXRCE.sh` from this repository (in the `helper_scripts` directory) to your `PX4-Autopilot` folder. Running this script will activate the agent and provide access to the Mavlink ROS2 messages.
18. Copy the `gz_camera.yaml` file from this repository (in the `helper_scripts` directory) to your root directory.
19. In the root directory, a ROS2 workspace must be created.
```
   mkdir -p <pick ws name>/src
```
20. Finally, install the necessary PX4 packages.
```
   cd <pick ws name>/src
   git clone https://github.com/PX4/px4_ros_com.git -b release/v1.14 --recursive
   git clone https://github.com/PX4/px4_msgs.git -b release/1.14 --recursive
   cd ..
   colcon build
   source install/setup.bash
```
NOTE: It does not specifically matter which branch is used of the above repositories, just that they are both the same.

To test the setup, you will need three terminals (all with ROS2 properly sourced). In the first terminal, navigate to your `PX4-Autopilot` folder and run the following commands:
```
   ./set_GZ_SIM_RESOURCE_PATH.sh
   ./run_baylands_total.sh
```
At this point, the Gazebo window should appear with the vehicle in the same park-like setting as before.

In the second terminal, navigate to your `PX4-Autopilot` folder and run the following command to activate the MicroXRCE agent:
```
   ./start_uXRCE.sh
```
You should see it begin to activate and different topics appear.

In the third terminal, run `ros2 topic list` and verify that the Mavlink topics appear.
