import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """A simple ROS 2 node that periodically prints 'Hello world'."""

    def __init__(self) -> None:
        super().__init__('hello_node')
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('HelloNode has been started.')

    def timer_callback(self) -> None:
        self.get_logger().info('Hello world from ROS 2 (Python)!')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
