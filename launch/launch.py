import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time':True},
                        {'yaml_filename': '/home/robot/projects/mobile_robotics/ws02/src/mr_nav2/config/map/cave/map.yaml'} 
                        ]),
        
        Node(
            
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time':True}],
            output='screen',

            ),
        
        Node(
        
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time':True},
                        {'autostart': True},
                        {'node_names': ['map_server']}
                        ]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_map',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '/world', '/map']
        )

    ])