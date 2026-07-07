cmake_minimum_required(VERSION 3.8)
project(wtb_vehicle_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/cleanup_autoware_sim.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
