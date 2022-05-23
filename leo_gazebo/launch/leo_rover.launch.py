import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_leo_description = get_package_share_directory('leo_rover_description')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/leo_rover.urdf'

    urdf = os.path.join(
      pkg_leo_description,
      urdf_file_name)

    print("urdf_file_name : {}".format(urdf))

    robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf])


    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'leo_rover', '-topic', '/robot_description'],
                        output='screen')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        gazebo,
        robot_state_pub,
        spawn_entity
  ])