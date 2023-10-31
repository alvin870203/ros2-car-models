source /opt/ros/humble/setup.bash 
rm -rf build/* install/* log/*
colcon build --symlink-install
source install/setup.bash 
sudo apt-cache policy ros2-controllers
sudo apt-cache search ros2-controllers
sudo apt-cache policy ros-humble-ros2-controllers
ros2 launch gazebo_ros gazebo.launch.py 
sudo apt-cache search ros2-control
sudo apt-cache search ros2-control | grep humble
source install/setup.bash 
ros2 launch description publish_robot_state.launch.py
source install/setup.bash 
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity autocar
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
clear
source /opt/ros/humble/setup.bash 
ros2 launch gazebo_ros gazebo.launch.py --show-arguments
sudo apt-cache search gazebo-ros-pkgs
sudo apt-cache policy ros-humble-gazebo-ros-pkgs
ros2 launch gazebo_ros gazebo.launch.py --print-description 
source /opt/ros/humble/setup.bash 
