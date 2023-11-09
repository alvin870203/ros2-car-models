from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo',
            executable='odom2tf',
            name='odom2tf',
            parameters=[
                {'use_sim_time': True}
            ],
            output='screen',
        ),
    ])
