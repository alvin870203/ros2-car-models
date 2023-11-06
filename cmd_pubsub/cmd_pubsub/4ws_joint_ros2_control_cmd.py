import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.axle_publisher_ = self.create_publisher(Float64MultiArray,
                                                     'axle_joints_velocity_controller/commands',
                                                     10)
        self.steer_publisher_ = self.create_publisher(Float64MultiArray,
                                                      'steer_joints_position_controller/commands',
                                                      10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        axle_velocity_cmd = Float64MultiArray(data=[2.0, 2.0, 2.0, 2.0])
        steer_position_cmd = Float64MultiArray(data=[-0.5, -0.5, -0.5, -0.5])

        self.axle_publisher_.publish(axle_velocity_cmd)
        self.steer_publisher_.publish(steer_position_cmd)
        self.get_logger().info('Publishing: axle_velocity_cmd & steer_position_cmd')


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