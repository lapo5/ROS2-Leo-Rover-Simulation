# Leo Rover Simulation

Simulation for the Leo Rover (Gazebo)

- Leo Gazebo: launchers, worlds

- Leo Gazebo Plugins: plugin for the gazebo model. 
	- mimic_joint_plugin: to couple front and rear wheel on each side
	- differential_plugin: to emulate passive boogie

- Mars World (leo_gazebo_worlds): Mars World from MIT LEO Competitions

- Leo Rover Description: fork of ROS2 package, to work with Gazebo ROS2

## Launch Simulation

### Standard Simulation with Empty World

ros2 launch leo_gazebo leo_rover.launch.py

### Simulate LEO on Mars

ros2 launch leo_gazebo leo_rover_on_mars.launch.py

## Bridge Real Robot with Simulation with ROS Bridge

### Create ROS1 msgs ws (ros1_msgs), with src folder inside

copy leo_msgs (ros1)

cd ..

ros_source

catkin_make_isolated --install

### Create ROS2 msgs ws (ros2_msgs), with src folder inside

copy leo_msgs (ros2)

cd ..

ros2_source

colcon build

### Create BRIDGE ws

git clone -b foxy https://github.com/ros2/ros1_bridge.git

cd ..

#### NOTE: ROS1 Bridge has a problem with the controller-managaer-msgs (may purge other packages!)
If needed, re-install them after the compilation!!!!!

sudo apt remove ros-foxy-controller-manager-msgs

### Compile ROS Bridge
source /opt/ros/noetic/setup.bash

source /opt/ros/foxy/setup.bash 

source ../ros1_msgs/install_isolated/setup.bash

source ../ros2_msgs/install/setup.bash

colcon build --cmake-force-configure

### Check leo msgs are mapped
source install/setup.bash

ros2 run ros1_bridge dynamic_bridge --print-pairs

### Start the Bridge
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics

