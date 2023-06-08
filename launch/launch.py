import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration

def generate_launch_description(): 

    this_directory = get_package_share_directory('stage_ros2')

    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='cave'),
        description='World file relative to the project world file, without .world')

    def stage_world_configuration(context):
        file = os.path.join(
            this_directory,
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)
    
    return LaunchDescription([
        stage_world_arg,
        stage_world_configuration_arg,
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{
                "world_file": [LaunchConfiguration('world_file')]}],
        ),
        # Node(
        #     package='mr_goto',
        #     executable='goto',
        #     name='mynode',
        #     output='screen'),
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