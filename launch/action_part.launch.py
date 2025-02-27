from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capstone_turtlebot_controller',
            namespace='Lima',
            executable='turtlebot_controller',
            output='screen',
            parameters=[{'robot_name': 'Lima',
                         'scan_name' : '/Lima/scan',
                         }]
        ),
        # Node(
        #     package='capstone_turtlebot_controller',
        #     namespace='Romeo',
        #     executable='turtlebot_controller',
        #     output='screen',
        #     parameters=[{'robot_name': 'Romeo',
        #                  'scan_name' : '/Romeo/scan',
        #                  }]
        # ), 
    ])