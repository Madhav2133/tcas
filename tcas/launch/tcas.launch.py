#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    ld = LaunchDescription()
    
    config1 = os.path.join(
        get_package_share_directory('tcas'),
        'config',
        'junction_details.yaml'
    )
    
    config2 = os.path.join(
        get_package_share_directory('tcas'),
        'config',
        'robot_details.yaml'
    )
    
    config3 = os.path.join(
        get_package_share_directory('multi_robot_demo'),
        'params',
        'tb3_0_planner_server.yaml'
    )
    
    node = Node(
        package='tcas',
        name='run_tcas',
        executable='run_tcas',
        parameters=[config1, config2, config3],
        output='screen'
    )
    
    ld.add_action(node)
    return ld