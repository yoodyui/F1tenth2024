from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('f1tenth_bringup'), 'config')
    configuration_basename = 'cartographer_config.lua'
    # Path to map .pbstream file
    pbstream_file = '/home/parallels/F1tenth2024/house.pbstream' 

    return LaunchDescription([
        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[
                {'use_sim_time': False},  
            ],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                # Use below line for localization on a map
                '-load_state_filename', pbstream_file
            ],
            remappings=[
                ('/scan', '/scan'),
            ]
        ),
        
        # Cartographer Occupancy Grid Node
        #Node(
        #    package='cartographer_ros',
        #    executable='cartographer_occupancy_grid_node',
        #    name='cartographer_occupancy_grid_node',
        #    output='screen',
        #    parameters=[{
        #        'use_sim_time': False
        #    }]
        # ),
    ])
