#!/usr/bin/env python
import os
from launch import LaunchDescription, LaunchContext
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

prefix="tita_hw"

def generate_launch_description():
    declared_arguments = []

    hardware_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hardware_bridge'),
            'launch',
            'hardware_bridge.launch.py'
        )),
        launch_arguments={
            'urdf': "robot.xacro",
            'yaml_path': "hw_bringup",
        }.items(),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )
    
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", prefix+"/controller_manager"],
    )

    
    effort_controller = Node(
        package='controller_manager',
        # output='screen',
        executable='spawner',
        arguments=["effort_controller", "--controller-manager", prefix+"/controller_manager"],
    )

    return LaunchDescription(declared_arguments + [
        hardware_controller_manager_launch,

        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        effort_controller,
 ])
