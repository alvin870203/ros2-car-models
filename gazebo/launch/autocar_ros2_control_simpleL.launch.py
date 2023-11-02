from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_ros2_control = "true" # Use ros2_control instead of gazebo_control

    world_file = PathJoinSubstitution(
        [FindPackageShare("gazebo"),
        "worlds",
        "simpleL.world"],
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo"),
        "launch",
        "gazebo.launch.py"],
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file, 'use_ros2_control': use_ros2_control}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_sim)

    return ld
