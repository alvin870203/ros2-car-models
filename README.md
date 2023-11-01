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
ros2 launch viz view_model.launch.py
# Run `rviz2 -d rviz2_config_file_path` to just open rviz2 with saved settings.
```

## Spawn URDF in Gazebo
```bash
source install/setup.bash

ros2 launch gazebo gazebo.launch.py # Launch robot in empty Gazebo

# Or,
ros2 launch gazebo autocar_simpleL.launch.py # Launch robot in example saved world of Gazebo

# Or, underline commands of launching robot in empty Gazebo
ros2 launch description description.launch.py # Publish /robot_description
ros2 launch gazebo_ros gazebo.launch.py # Launch empty Gazebo (or by running `gazebo --verbose -s libgazebo_ros_factory.so`)
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity robot_name # Spawn robot in Gazebo (or spawn robot SDF by running `ros2 run gazebo_ros spawn_entity.py -file sdf_file_path -entity robot_name`)
```

## Send test `/cmd_vel` to control simulated robot
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or,
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/target_namespace # Specify the namespace (to publish to /target_namespace/cmd_vel)

# Or,
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/target_namespace/target_topic # Remap the topic (to publish to /target_namespace/target_topic)
```

## View simulated robot in rviz2
```bash
source install/setup.bash
ros2 launch viz view_robot.launch.py
```