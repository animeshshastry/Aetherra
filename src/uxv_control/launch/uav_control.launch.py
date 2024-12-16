#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions               import LaunchConfiguration as LC
from launch.conditions                  import IfCondition
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import PythonExpression
from launch_ros.actions                 import Node

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('namespace',                  default_value='alfa'),

        ################### UXV Control ######################

        Node(
            package='uxv_control',
            executable='uav_control.py',
            namespace=[LC('namespace')],
            output='both',
            remappings=[
                ('/goal_path',('/',LC('namespace'),'/goal_path')),
                ('/raw_odom',('/',LC('namespace'),'/raw_odom')),
                ('/cmd_vel',('/',LC('namespace'),'/cmd_vel')),
            ],
            # additional_env={
            #     'ROS_DOMAIN_ID': LC('domain_id'),
            # },
        ),
    ])
