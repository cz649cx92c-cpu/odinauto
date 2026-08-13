#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/orangepi/ugv/install/setup.bash
source /home/orangepi/ugv/ros2_ws/install/setup.bash
source /home/orangepi/ugv/odinauto/SRU-Odin/Deployment_ros2/install/setup.bash

cd /home/orangepi/ugv/odinauto/goal_ui
exec /usr/bin/python3 server.py
