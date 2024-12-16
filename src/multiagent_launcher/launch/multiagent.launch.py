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

    enable_pos_ctrl = 'True'
    enable_vio = 'False'

    robots_file = open(os.path.join(get_package_share_directory('multiagent_launcher'),'config', 'robots.yaml'))
    robots = yaml.safe_load(robots_file)

    launch_description_list=[]
    for robot in robots:
            namespace = robot['name']     
            launch_description_list.extend([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('uxv_control'),'launch', 'uav_control.launch.py')
                    ),
                    condition=IfCondition(enable_pos_ctrl),
                    launch_arguments={
                        'namespace': namespace,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('ov_msckf'),'launch', 'vins.launch.py')
                    ),
                    condition=IfCondition(enable_vio),
                    launch_arguments={
                        'namespace': namespace,
                        'vio_camera': 'tracking_down',
                        'vio_config': os.path.join(get_package_share_directory('multiagent_launcher'), 'config', 'vio_configs', 'alfa', 'estimator_config.yaml'),
                    }.items(),
                ),
            ])
    
    ld = LaunchDescription()
    ld.add_action(GroupAction(launch_description_list))
    return ld

