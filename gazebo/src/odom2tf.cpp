#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("odom2tf_publisher")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to a /p3d/odom topic and call handle_robot_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/p3d/odom";
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_robot_pose, this, std::placeholders::_1));
  }

private:
  void handle_robot_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";

    // Car only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // For the same reason, car can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}