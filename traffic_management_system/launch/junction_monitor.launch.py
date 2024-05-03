#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    ld = LaunchDescription()
    
    config1 = os.path.join(
        get_package_share_directory('traffic_management_system'),
        'config',
        'junction_details.yaml'
    )
    
    config2 = os.path.join(
        get_package_share_directory('traffic_management_system'),
        'config',
        'robot_details.yaml'
    )
    
    node = Node(
        package='traffic_management_system',
        name='junction_monitor',
        executable='junction_monitor',
        parameters=[config1, config2],
        output='screen'
    )
    
    ld.add_action(node)
    return ld