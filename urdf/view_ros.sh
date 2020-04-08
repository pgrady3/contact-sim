#!/bin/bash
eval "$(conda shell.bash hook)"
conda deactivate
conda deactivate
source /opt/ros/kinetic/setup.bash
roslaunch urdf_tutorial display.launch model:=hand_ros.urdf
