import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW

import json

# Created python modules
from .collision import test_me
from .collision import json_to_dict
from .collision import calculate_new_coordinates
from .collision import get_advise


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_DOT_py')
        # Subscriber to get current air traffic over Fyn
        self.air_traffic_subscription = self.create_subscription(
            String,
            'air_traffic',
            self.air_traffic_callback,
            10
        )
        self.air_traffic_subscription  # prevent unused variable warning

        # Subscriber to get the position of the drone
        self.drone_lat_lon_alt_subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.NavSatFix_callback,
            qos.qos_profile_sensor_data
        )
        self.drone_lat_lon_alt_subscription  # prevent unused variable warning

        # Subscriber to get the velocity of the drone
        self.drone_velocity_subscription = self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.GPSRAW_callback,
            qos.qos_profile_sensor_data
        )
        self.drone_velocity_subscription  # prevent unused variable warning

        # Subscriber to get the heading of the drone
        self.drone_velocity_subscription = self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.compass_hdg_callback,
            qos.qos_profile_sensor_data
        )
        self.drone_velocity_subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(String, '/new_topic', 10)


        self.drone_info = {}

        # Timer to publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_msg)

    # TODO load in the msg to the airtraffic dict
    def air_traffic_callback(self, air_traffic_msg):
        self.air_traffic = json.loads(air_traffic_msg.data)


    # TODO load in the msg to the drone dict
    def NavSatFix_callback(self, NavSatFix_msg):
        self.drone_info['lat'] = NavSatFix_msg.latitude
        self.drone_info['lon'] = NavSatFix_msg.longitude
        self.drone_info['alt'] = NavSatFix_msg.altitude

    # TODO load in the msg to the drone dict
    def GPSRAW_callback(self, GPSRAW_msg):
        self.drone_info['speed'] = GPSRAW_msg.vel

    def compass_hdg_callback(self, compass_hdg_msg):
        self.drone_info['heading'] = compass_hdg_msg.data

    # TODO publish to new topic
    def publish_msg(self):

        msg = String()
        # msg.data = 'test'
        msg.data = get_advise(self.air_traffic, self.drone_info)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing message to new_topic: \n{msg.data}' )


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
    test_me()
    main()