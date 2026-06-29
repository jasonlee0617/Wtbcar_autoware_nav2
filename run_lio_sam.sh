#!/usr/bin/env bash

set -e

source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/Wtbcar_autoware_nav2/install/setup.bash

ros2 launch lio_sam run_wtb_mapping.launch.py \
  launch_hardware:=true \
  launch_rviz:=true
