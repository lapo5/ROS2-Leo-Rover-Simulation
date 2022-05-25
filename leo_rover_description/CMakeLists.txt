cmake_minimum_required(VERSION 3.5)
project(leo_rover_description)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY
    config
    models
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh")

ament_package()
