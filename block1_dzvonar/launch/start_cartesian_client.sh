#!/bin/bash
# Script to start the cartesian control client in a new terminal
# Run this in a separate terminal after starting the main launch file

source /opt/ros/jazzy/setup.bash
source /home/imrich/ros2-ws-rrm/install/setup.bash
ros2 run block1_dzvonar cartesian_control_client
