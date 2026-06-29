#!/usr/bin/env bash

set -e

source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/Wtbcar_autoware_nav2/install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/eisa/autoware_map/maps/wtb_map4 \
  vehicle_model:=wtb_vehicle \
  sensor_model:=wtb_sensor_kit \
  data_path:=/home/eisa/autoware_data
