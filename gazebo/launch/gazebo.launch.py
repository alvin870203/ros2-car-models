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
    prefix = LaunchConfiguration('prefix')

    # ALC231031 BEGIN - TODO: uncomment and revise these when ros2_control is set.
    # config_robot_velocity_controller = PathJoinSubstitution(
    #     [FindPackageShare("control"), "config", "control.yaml"]
    # )
    # ALC231031 END

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("description"), "urdf", "autocar.urdf.xacro"]
            ),
            " ",
            "name:=autocar",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            # ALC231031 BEGIN - TODO: uncomment and revise these when ros2_control is set.
            # " ",
            # "gazebo_controllers:=",
            # config_robot_velocity_controller,
            # ALC231031 END
        ]
    ).perform(LaunchContext())
    robot_description = {"robot_description": robot_description_content}

    # ALC231031 BEGIN - TODO: uncomment and revise these when ros2_control is set.
    # spawn_robot_velocity_controller = Node(
    #     package='controller_manager',
    #     executable='spawner.py',
    #     arguments=['robot_velocity_controller', '-c', '/controller_manager'],
    #     output='screen',
    # )
    # ALC231031 END

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    # ALC231031 BEGIN - TODO: uncomment and revise these when ros2_control is set.
    # spawn_joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner.py',
    #     arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    #     output='screen',
    # )
    # ALC231031 END

    # ALC231031 BEGIN - TODO: uncomment and revise these when ros2_control is set.
    # # Make sure spawn_robot_velocity_controller starts after spawn_joint_state_broadcaster
    # diffdrive_controller_spawn_callback = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_joint_state_broadcaster,
    #         on_exit=[spawn_robot_velocity_controller],
    #     )
    # )
    # ALC231031 END

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
    # ld.add_action(spawn_joint_state_broadcaster)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.
    # ld.add_action(diffdrive_controller_spawn_callback)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    # ld.add_action(launch_robot_control)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.
    # ld.add_action(launch_robot_teleop_base)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.

    return ld
