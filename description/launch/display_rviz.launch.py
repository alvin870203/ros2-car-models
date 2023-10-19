import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('description'))
    xacro_file = os.path.join(pkg_path,'urdf','yahboomcar_R2.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path,'rviz','yahboomcar_R2.rviz')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        # condition=IfCondition(LaunchConfiguration('gui'))
    )


    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        # node_joint_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])
