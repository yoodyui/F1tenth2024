from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription, ExecuteProcess  # Added ExecuteProcess here
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Path to the sllidar_ros2 view_sllidar_s1_launch.py
    sllidar_launch_file_dir = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'view_sllidar_s1_launch.py')

    # Path to Cartographer config
    cartographer_config_dir = os.path.join(get_package_share_directory('f1tenth_bringup'), 'config')
    configuration_basename = 'cartographer_config.lua'
    # Path to cartographer_launch.py and nav2_launch.py in f1tenth_bringup
    f1tenth_launch_dir = os.path.join(get_package_share_directory('f1tenth_bringup'), 'launch')

    #Path to new bringup launch to avoid using map.yaml
    bringup_dir = get_package_share_directory('f1tenth_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Include the sllidar_ros2 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_file_dir),
        ),
        
        # Cartographer Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([f1tenth_launch_dir, '/cartographer_launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                'configuration_basename': 'cartographer_config.lua',  # Your Cartographer config
            }.items(),
        ),
        
        # Navigation node (will be calling map.yaml--you need to turn off occupany node)
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([f1tenth_launch_dir, '/nav2_launch.py']),
        #    launch_arguments={
        #        'params_file': params_file,
        #        'use_sim_time': 'false',
        #    }.items(),
        #),
        
        # Static transform publisher from base_link to laser frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.1', '0', '0.2', '3.14159', '0', '0', 'base_link', 'laser']
        ),
        
        # Joy node
        #Node(
        #    package='joy',
        #    executable='joy_node',
        #    name='joy_node',
        #    output='screen'
        #),
        
        # Teleop Twist Joy node (Python launch file)
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(
        #            get_package_share_directory('teleop_twist_joy'),
        #            'launch',
        #            'teleop-launch.py'
        #        )
        #    )
        #),
        
        # Adding micro-ROS agent for ODOM node
        #ExecuteProcess(
        #    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
        #    output='screen'
        #),

        # VESC node (Python launch file)
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(
        #            get_package_share_directory('vesc_driver'),
        #            'launch',
        #            'vesc_driver_node.launch.py'
        #        )
        #    )
        #),
        
        # Ackermann node (XML launch file)
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('vesc_ackermann'),
                    'launch',
                    'vesc_to_odom_node.launch.xml'
                )
            )
        ),
    ])
