#!/bin/bash

source set_GZ_SIM_RESOURCE_PATH.sh
source ../ros2_gzbridge/install/setup.bash

 # cd/home/mrossmann/ros2_workspace/PX4-Autopilot/
 
 PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth
