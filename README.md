![Alt text](rviz2__winstxnhdw__AutoCarROS2.png)

# Note

## Remember `source /opt/ros/humble/setup.bash` when starting every terminal

## Create a C++ ROS2 package

```bash
cd /home/ws/src/
ros2 pkg create --build-type ament_cmake <package_name>
```

## Build a C++ ROS2 package

```bash
cd /home/ws/
colcon build --symlink-install
```

## Preview URDF in rviz2 (Cannot work with Gazebo)
> Just for making sure the parts in the urdf that are unrelated to Gazebo and control are correct.
```bash
source install/setup.bash
ros2 launch description preview_urdf.launch.py
# Run `rviz2 -d rviz2_config_file_path` to just open rviz2 with saved settings.
```

## Spawn URDF in Gazebo
```bash
source install/setup.bash
ros2 launch description publish_robot_state.launch.py # Publish /robot_description
ros2 launch gazebo_ros gazebo.launch.py # Lauch empty Gazebo (or by running `gazebo --verbose -s libgazebo_ros_factory.so`)
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity robot_name # Spawn robot in Gazebot (or spawn robot SDF by running `ros2 run gazebo_ros spawn_entity.py -file sdf_file_path -entity robot_name`)
```

## Send test `/cmd_vel` to control simulated robot
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard # or specify the namespace (to publish to /target_namespace/cmd_vel) by running `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/target_namespace`
```