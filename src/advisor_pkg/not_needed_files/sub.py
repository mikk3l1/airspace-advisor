import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mavros_msgs.msg import WaypointList


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for wayPoint in msg.waypoints:
            print(wayPoint)
            self.get_logger().info(f'I heard:\nlat:{wayPoint.x_lat}\nlong:{wayPoint.y_long}')
        print('----------------------------------------------')
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()