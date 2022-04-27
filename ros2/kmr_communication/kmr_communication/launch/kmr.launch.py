import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(argv=sys.argv[1:]):
    pkg_name = 'kmr_communication'

    config_file_path = os.path.join(
            get_package_share_directory(pkg_name),
            'config',
            'bringup.yaml')
    config_file = open(config_file_path)
    parsed_config_file = yaml.load(config_file, Loader=yaml.FullLoader)
    
    connection_params = parsed_config_file['connection_params']['ros__parameters']
    
    connection_type = connection_params['connection_type']
    robot = connection_params['robot']

    param_dir = LaunchConfiguration(
        'param_dir',
        default=config_file_path
    )

    return LaunchDescription([
        DeclareLaunchArgument(
                'param_dir',
                default_value=param_dir,
                description="Full path to parameter file to load"
            ),

        Node(
            package=pkg_name,
            executable='lbr',
            name='lbr_command_node',
            output='screen',
            emulate_tty=True,
            arguments=['-c', connection_type, '-ro', robot],
            parameters=[param_dir]
        ), 

        Node(
            package=pkg_name,
            executable='kmp',
            name='kmp_command_node',
            output='screen',
            emulate_tty=True,
            arguments=['-c', connection_type, '-ro', robot],
            parameters=[param_dir]
        ),
        #add Daniel
        Node(
            package=pkg_name,
            executable='kmp',
            name='kmp_odometry_node',
            output='screen',
            emulate_tty=True,
            arguments=['-c', connection_type, '-ro', robot],
            parameters=[param_dir]
        ),
         Node(
            package=pkg_name,
            executable='kmp',
            name='kmp_laserscan_node',
            output='screen',
            emulate_tty=True,
            arguments=['-c', connection_type, '-ro', robot],
            parameters=[param_dir]
        ),
        #end Daniel

        Node(
            package=pkg_name,
            executable='camera',
            name='camera_node',
            output='screen',
            emulate_tty=True,
            arguments=['-ro', robot],
            parameters=[param_dir]
        ),
        
    ])