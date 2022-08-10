cmake_minimum_required(VERSION 3.5)
project(leo_gazebo_worlds)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

install(
  DIRECTORY
    media
    models
    worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()