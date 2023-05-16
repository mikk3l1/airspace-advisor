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
from .col_test import get_new_coordinates
from .col_test import calculate_new_coordinates_using_nautical_miles
from .collision import get_advise


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_py')
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

        self.advise_publisher = self.create_publisher(String, '/advise', 10)
        self.air_traffic_new_coordinates_publisher = self.create_publisher(String, '/air_traffic_new_coordinates', 10)
        self.drone_new_coordinates_publisher = self.create_publisher(String, '/drone_new_coordinates', 10)

        self.drone_info = {}
        # self.air_traffic = {}

        # Timer to publish every 2 seconds
        self.advise_timer = self.create_timer(2.0, self.publish_advise)
        self.air_traffic_new_coordinates_timer = self.create_timer(2.0, self.publish_air_traffic_new_coordinates)
        self.drone_new_coordinates_timer = self.create_timer(2.0, self.publish_drone_new_coordinates)

    
    def air_traffic_callback(self, air_traffic_msg):
        self.air_traffic = json.loads(air_traffic_msg.data)

    def NavSatFix_callback(self, NavSatFix_msg):
        self.drone_info['lat'] = NavSatFix_msg.latitude
        self.drone_info['lon'] = NavSatFix_msg.longitude
        self.drone_info['alt'] = NavSatFix_msg.altitude

    def GPSRAW_callback(self, GPSRAW_msg):
        drone_speed_in_knots = GPSRAW_msg.vel * 0.01944   # convert cm/s to knots
        self.drone_info['speed'] = drone_speed_in_knots
        self.drone_info['speed_in_cm'] = GPSRAW_msg.vel

    def compass_hdg_callback(self, compass_hdg_msg):
        self.drone_info['heading'] = compass_hdg_msg.data
        self.drone_info['track'] = compass_hdg_msg.data

    # This is the old advise(r)
    def publish_advise(self):
        msg = String()
        msg.data = get_advise(self.air_traffic, self.drone_info)
        self.advise_publisher.publish(msg)
        self.get_logger().info(f'Publishing advise: \n{msg.data}')

    def publish_air_traffic_new_coordinates(self):
        msg = String()
        for entry in self.air_traffic:
            if self.air_traffic[entry].get('lat') == None:
                continue
            # self.air_traffic[entry]['new_coordinates'] = get_new_coordinates(self.air_traffic[entry])
            self.air_traffic[entry]['new_coordinates'] = calculate_new_coordinates_using_nautical_miles(self.air_traffic[entry])

        msg.data = json.dumps(self.air_traffic, indent=1)
        self.air_traffic_new_coordinates_publisher.publish(msg)
        self.get_logger().info(f'Publishing air traffic with new coordinates:\n{msg.data}')

    def publish_drone_new_coordinates(self):
        msg = String()

        # self.drone_info['new_coordinates'] = get_new_coordinates(self.drone_info)
        self.drone_info['new_coordinates'] = calculate_new_coordinates_using_nautical_miles(self.drone_info)

        msg.data = json.dumps(self.drone_info, indent=1)
        self.drone_new_coordinates_publisher.publish(msg)
        self.get_logger().info(f'Publishing drone_info with new coordinates:\n{msg.data}')


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