#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("autocar/set_joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {

        auto joint_cmd = trajectory_msgs::msg::JointTrajectory();
        joint_cmd.header.frame_id = "base_link";
        joint_cmd.points.resize(1);

        joint_cmd.joint_names.push_back("l_steer");
        joint_cmd.points[0].positions.push_back(-0.5);

        joint_cmd.joint_names.push_back("r_steer");
        joint_cmd.points[0].positions.push_back(-0.5);

        joint_cmd.joint_names.push_back("fl_axle");
        joint_cmd.points[0].positions.push_back(0.0);

        joint_cmd.joint_names.push_back("fr_axle");
        joint_cmd.points[0].positions.push_back(0.0);

        joint_cmd.joint_names.push_back("bl_axle");
        joint_cmd.points[0].positions.push_back(0.0);

        joint_cmd.joint_names.push_back("br_axle");
        joint_cmd.points[0].positions.push_back(0.0);

        RCLCPP_INFO(this->get_logger(), "Publishing: joint_cmd");
        publisher_->publish(joint_cmd);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}