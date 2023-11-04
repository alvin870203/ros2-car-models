import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'autocar/set_joint_trajectory', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        joints_cmd = JointTrajectory()

        joints_cmd.header.frame_id = "base_link"
        joints_cmd.joint_names = ["l_steer",
                                  "r_steer",
                                  "fl_axle",
                                  "fr_axle",
                                  "bl_axle",
                                  "br_axle"]
        joints_cmd.points.append(JointTrajectoryPoint(positions=[-0.5,
                                                                 -0.5,
                                                                 0.0,
                                                                 0.0,
                                                                 0.0,
                                                                 0.0]))

        self.publisher_.publish(joints_cmd)
        self.get_logger().info('Publishing: joints_cmd')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()