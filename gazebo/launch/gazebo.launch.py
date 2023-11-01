from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('description')).
                                                    parent.resolve())])

    # Launch args
    world_path = LaunchConfiguration('world_path')

    config_robot_controller = PathJoinSubstitution(
        [FindPackageShare("control"), "config", "control.yaml"]
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
            "gazebo_controllers:=",
            config_robot_controller,
        ]
    ).perform(LaunchContext())
    robot_description = {"robot_description": robot_description_content}

    spawn_ackermann_steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '-c', '/controller_manager'],
        output='screen',
    ) # ALC231101 - TODO: Remap /ackermann_steering_controller/odometry:=/odom

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_ackermann_steering_controller starts after spawn_joint_state_broadcaster
    ackermann_steering_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_ackermann_steering_controller],
        )
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=['-entity',
                   'autocar',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    # ALC231031 BEGIN - TODO: uncomment and revise these when ros2_control is set.
    # # Launch robot_control/control.launch.py which is just robot_localization.
    # launch_robot_control = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution(
    #     [FindPackageShare("robot_control"), 'launch', 'control.launch.py'])))
    #
    # # Launch robot_control/teleop_base.launch.py which is various ways to tele-op
    # # the robot but does not include the joystick. Also, has a twist mux.
    # launch_robot_teleop_base = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution(
    #     [FindPackageShare("robot_control"), 'launch', 'teleop_base.launch.py'])))
    # ALC231031 END

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(ackermann_steering_controller_spawn_callback)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    # ld.add_action(launch_robot_control)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.
    # ld.add_action(launch_robot_teleop_base)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.

    return ld
