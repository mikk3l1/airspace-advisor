import rclpy
from rclpy.node import Node
from rclpy import qos

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW


class MAVROS_Subscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.drone_lat_lon_alt_subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.NavSatFix_callback,
            qos.qos_profile_sensor_data
        )
        self.drone_velocity_subscription = self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.GPS_callback,
            qos.qos_profile_sensor_data
        )
        # self.subscription  # prevent unused variable warning

    def NavSatFix_callback(self, msg):
        # self.get_logger().info(f'I heard: \nLat: {msg.latitude} \nLon: {msg.longitude} \nAlt: {msg.altitude}')
        pass

    def GPS_callback(self, msg):
        self.get_logger().info(f'I heard: \nvel: {msg.vel}')


def main(args=None):
    rclpy.init(args=args)

    MAVROS_subscriber = MAVROS_Subscriber()

    rclpy.spin(MAVROS_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    MAVROS_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()