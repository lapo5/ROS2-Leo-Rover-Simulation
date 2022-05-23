cmake_minimum_required(VERSION 3.5)
project(leo_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(leo_gazebo_differential_plugin SHARED
  src/differential_plugin.cpp
)


add_library(mimic_joint_plugin SHARED src/mimic_joint_plugin.cc)

target_link_libraries(mimic_joint_plugin ${GAZEBO_LIBRARIES})



ament_target_dependencies(leo_gazebo_differential_plugin
  gazebo_dev
  gazebo_ros
)
ament_target_dependencies(mimic_joint_plugin
  gazebo_dev
  gazebo_ros
)

ament_export_libraries(leo_gazebo_differential_plugin)

ament_export_libraries(mimic_joint_plugin)

ament_package()

install(
  TARGETS leo_gazebo_differential_plugin mimic_joint_plugin
  LIBRARY DESTINATION lib
)