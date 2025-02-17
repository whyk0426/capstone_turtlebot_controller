from launch_ros.substitutions import FindPackageShare

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node

import math

use_sim_time = LaunchConfiguration('use_sim_time', default='false')
resolution = LaunchConfiguration('resolution', default='0.05')
publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

PI = math.pi

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('capstone_turtlebot_controller'),
                    'launch',
                    'Lima_cartographer.launch.py'
                ])
            ),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('capstone_turtlebot_controller'),
        #             'launch',
        #             'Alpha_cartographer.launch.py',
        #         ])
        #     ),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('capstone_turtlebot_controller'),
                    'launch',
                    'Romeo_cartographer.launch.py',
                ])
            ),
        ),
    
        # Node(
        #     package='capstone_turtlebot_controller',
        #     namespace='Lima',
        #     executable='cmd_publisher_node',
        #     output='screen',
        #     parameters=[{'goal1': [0.5, 0, PI, 0, 0, 0]},
        #                 #{'goal2': [0, 0, 0]},
        #                 {'robot_name': 'Lima'},                        
        #                 ]
        # ),
        # Node(
        #     package='capstone_turtlebot_controller',
        #     namespace='Alpha',
        #     executable='cmd_publisher_node',
        #     output='screen',
        #     parameters=[{'goal1': [0.5, 0.5, PI, 0, 0.5, 0]},
        #                 {'robot_name': 'Alpha'},  
        #                 ]
        # ),
        # Node(
        #     package='capstone_turtlebot_controller',
        #     namespace='Romeo',
        #     executable='cmd_publisher_node',
        #     output='screen',
        #     parameters=[{'goal1': [1.5, 0.5, PI, 1, 0.5, 0]},
        #                 {'robot_name': 'Romeo'},  
        #                 ]
        # ),

        Node(
            package='capstone_map_merging',
            executable='merge_map',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ("/map0", "/Lima/map"),
                #("/map1", "/Alpha/map"),
                ("/map1", "/Romeo/map"),
                ],
        ),

   

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'Lima_map']
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     output='screen',
        #     #arguments=['0', '0.5', '0', str(PI/4), '0', '0', 'map', 'Alpha_map'],
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'Alpha_map']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            #arguments=['0', '0', '0', str(PI/4), '0', '0', 'map', 'Romeo_map']
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'Romeo_map']
        ),
    ])
