from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution, AndSubstitution, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
    DeclareLaunchArgument('use_ros2_control', default_value='false',
                          description='Use ros2_control (:=true) or gazebo_control (:=false), by default is gazebo_control'),
    DeclareLaunchArgument('ros2_control_config', default_value='ackermann_control.yaml',
                          description='The file name of the ros2_control config, by default is ackermann_control.yaml'),
]


def generate_launch_description():

    # Launch args
    world_path = LaunchConfiguration('world_path')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    ros2_control_config = LaunchConfiguration('ros2_control_config')

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
            "use_ros2_control:=",
            use_ros2_control,
            " ",
            "gazebo_controllers:=",
            config_robot_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('description')).
                                                    parent.resolve())])

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

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ros2_control),
    )

    spawn_ackermann_steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(AndSubstitution(use_ros2_control,
                                              PythonExpression(["'", ros2_control_config, "'",
                                                                " == 'ackermann_control.yaml'"]))),
    ) # ALC231101 - TODO: Remap /ackermann_steering_controller/odometry:=/odom

    # Relay odom tf topic because it can not be remapped by ackermann_steering_controller
    relay_topic_to_tf_node = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'],
        output='screen',
        condition=IfCondition(AndSubstitution(use_ros2_control,
                                              PythonExpression(["'", ros2_control_config, "'",
                                                                " == 'ackermann_control.yaml'"]))),
    )

    spawn_rear_joints_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rear_joints_velocity_controller', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(AndSubstitution(use_ros2_control,
                                              PythonExpression(["'", ros2_control_config, "'",
                                                                " == 'joint_control.yaml'"]))),
    )

    spawn_front_joints_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['front_joints_velocity_controller', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(AndSubstitution(use_ros2_control,
                                              PythonExpression(["'", ros2_control_config, "'",
                                                                " == 'joint_control.yaml'"]))),
    )

    spawn_front_steers_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['front_steers_position_controller', '-c', '/controller_manager'],
        output='screen',
        condition=IfCondition(AndSubstitution(use_ros2_control,
                                              PythonExpression(["'", ros2_control_config, "'",
                                                                " == 'joint_control.yaml'"]))),
    )

    # Make sure all the other controller starts after spawn_joint_state_broadcaster
    controllers_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_ackermann_steering_controller,
                     spawn_rear_joints_velocity_controller,
                     spawn_front_joints_velocity_controller,
                     spawn_front_steers_position_controller],
        )
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
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(controllers_spawn_callback)
    ld.add_action(relay_topic_to_tf_node)
    # ld.add_action(launch_robot_control)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.
    # ld.add_action(launch_robot_teleop_base)  # ALC231031- TODO: uncomment and revise these when ros2_control is set.

    return ld
