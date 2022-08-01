import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class DistanceSubscriber(Node):

    def __init__(self):
        super().__init__('Distance subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'skippy_distance_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Distance travelled: "%f"' % msg.data)

class AngleSubscriber(Node):

    def __init__(self):
        super().__init__('Angle subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'skippy_yaw_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Z angle: "%f"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    distance_subscriber = DistanceSubscriber()
    angle_subscriber = AngleSubscriber()

    rclpy.spin(distance_subscriber)
    rclpy.spin(angle_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    distance_subscriber.destroy_node()
    angle_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
