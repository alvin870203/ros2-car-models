from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

ARGUMENTS = [
    DeclareLaunchArgument('use_4ws', default_value='false',
                          description='Use four-wheel steer (:=true) or front-wheel steer (:=false), by default is front-wheel steer'),
    DeclareLaunchArgument('use_ros2_control', default_value='false',
                          description='Use ros2_control (:=true) or gazebo_control (:=false), by default is gazebo_control'),
    DeclareLaunchArgument('ros2_control_config', default_value='ackermann_control.yaml',
                          description='The file name of the ros2_control config, by default is ackermann_control.yaml'),
    DeclareLaunchArgument('gazebo_control_config', default_value='autocar_ackermann_gazebo_control.xacro',
                          description='The file name of the gazebo_control config, by default is autocar_ackermann_gazebo_control.xacro'),
]


def generate_launch_description():

    # Launch args
    use_4ws = LaunchConfiguration('use_4ws')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    ros2_control_config = LaunchConfiguration('ros2_control_config')
    gazebo_control_config = LaunchConfiguration('gazebo_control_config')

    config_robot_controller = PathJoinSubstitution(
        [FindPackageShare("control"), "config", ros2_control_config]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("description"), "urdf", "autocar.urdf.xacro"]
            ),
            " ",
            "use_4ws:=",
            use_4ws,
            " ",
            "use_ros2_control:=",
            use_ros2_control,
            " ",
            "gazebo_controllers:=",
            config_robot_controller,
            " ",
            "gazebo_control_config:=",
            gazebo_control_config,
        ]
    )

    robot_description = {"robot_description": robot_description_content,
                         'use_sim_time': True}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)

    return ld
