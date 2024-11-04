#!/usr/bin/env python

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

from launch.actions import OpaqueFunction


prefix="tita_hw"
def launch_setup(context, *args, **kwargs):


    yaml_path = LaunchConfiguration('yaml_path')
    urdf = LaunchConfiguration('urdf')
    ctrl_mode = LaunchConfiguration('ctrl_mode')

    # save urdf
    robot_description_content_dir = PathJoinSubstitution(
        [FindPackageShare("tita_description"), "tita" , "xacro",  urdf]
    )
    xacro_executable = FindExecutable(name="xacro")

    robot_description_content = Command(
        [
            PathJoinSubstitution([xacro_executable]),
            " ",
            robot_description_content_dir,
            " ",
            "ctrl_mode:=",
            ctrl_mode,
            " ",
            "sim_env:=",
            "none",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    yaml_path_string = yaml_path.perform(context)

    robot_controllers = os.path.join(
        get_package_share_directory(yaml_path_string),
        "config",
        "controllers.yaml",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            robot_controllers,
        ],
        namespace=prefix, 
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        # output="both",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"frame_prefix": prefix+"/"},
        ],
        namespace=prefix, 
    )

    return [
        control_node,
        robot_state_pub_node,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="wbc",
            choices=["wbc", "sdk", "mcu"],
            description="Enable sdk of joint effort input",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf",
            default_value="robot.xacro",
            description="Define urdf file in description folder",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaml_path",
            default_value="hardware_bridge",
            description="Define yaml file folder",
        )
    )
    return LaunchDescription(
        declared_arguments  + 
        [OpaqueFunction(function=launch_setup)]
    )