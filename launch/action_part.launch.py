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
                         'goal_x': 1.0, 
                         'goal_y': 0.0
                         }]
        ),
        # Node(
        #     package='capstone_turtlebot_controller',
        #     namespace='Alpha',
        #     executable='turtlebot_controller',
        #     output='screen',
        #     parameters=[{'robot_name': 'Alpha',
        #                  'goal_x': 0.0, 
        #                  'goal_y': 0.0
        #                  }]
        # ),
        Node(
            package='capstone_turtlebot_controller',
            namespace='Romeo',
            executable='turtlebot_controller',
            output='screen',
            parameters=[{'robot_name': 'Romeo',
                         'goal_x': 0.0, 
                         'goal_y': 0.0
                         }]
        ),
    ])
