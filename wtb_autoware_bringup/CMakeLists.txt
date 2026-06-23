cmake_minimum_required(VERSION 3.8)
project(wtb_autoware_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
