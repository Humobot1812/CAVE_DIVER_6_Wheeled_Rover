from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='joy_teleop_simulation',
            executable='teleop_node',
            name='ps2_teleop',
            output='screen',
            parameters=[{'use_sim': True}]
        ),

    ])
