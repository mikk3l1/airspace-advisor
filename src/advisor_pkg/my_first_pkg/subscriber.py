import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW

import json

import sys

sys.path.append('../advisor_pkg')

# Created python modules
# from .collision import test_me

from my_first_pkg.calculate_new_coordinates import calculate_new_coordinates_using_nautical_miles

# from .calculate_new_coordinates import calculate_new_coordinates_using_nautical_miles

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('new_coordinates_node')

        self.declare_parameter('air_traffic_sub_param', value='/air_traffic')
        air_traffic_sub_name = self.get_parameter('air_traffic_sub_param').get_parameter_value().string_value

        self.declare_parameter('air_traffic_new_pub_param', value='/air_traffic_new_coordinates')
        air_traffic_new_pub_name = self.get_parameter('air_traffic_new_pub_param').get_parameter_value().string_value

        self.declare_parameter('drone_new_pub_param', value='/drone_new_coordinates')
        drone_new_pub_name = self.get_parameter('air_traffic_new_pub_param').get_parameter_value().string_value
        

        

        # Subscriber to get current air traffic over Fyn
        self.air_traffic_subscription = self.create_subscription(
            String,
            # '/air_traffic',
            air_traffic_sub_name,
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

        # self.advise_publisher = self.create_publisher(String, '/advise', 10)
        # self.air_traffic_new_coordinates_publisher = self.create_publisher(String, '/air_traffic_new_coordinates', 10)
        self.air_traffic_new_coordinates_publisher = self.create_publisher(String, air_traffic_new_pub_name, 10)
        # self.drone_new_coordinates_publisher = self.create_publisher(String, '/drone_new_coordinates', 10)
        self.drone_new_coordinates_publisher = self.create_publisher(String, drone_new_pub_name, 10)

        self.drone_info = {}
        self.air_traffic = {}

        # Timer to publish every 2 seconds
        self.air_traffic_new_coordinates_timer = self.create_timer(2.0, self.publish_air_traffic_new_coordinates)
        self.drone_new_coordinates_timer = self.create_timer(2.0, self.publish_drone_new_coordinates)

    
    def air_traffic_callback(self, air_traffic_msg):
        self.get_logger().info(f'received: \n{air_traffic_msg.data}')
        self.air_traffic = json.loads(air_traffic_msg.data)

    def NavSatFix_callback(self, NavSatFix_msg):
        self.get_logger().info(f'received: \n{NavSatFix_msg.latitude}, {NavSatFix_msg.longitude}, {NavSatFix_msg.altitude}')
        self.drone_info['lat'] = NavSatFix_msg.latitude
        self.drone_info['lon'] = NavSatFix_msg.longitude
        self.drone_info['alt'] = NavSatFix_msg.altitude

    def GPSRAW_callback(self, GPSRAW_msg):
        self.get_logger().info(f'received: \n{GPSRAW_msg.vel}')
        drone_speed_in_knots = GPSRAW_msg.vel * 0.01944   # convert cm/s to knots
        self.drone_info['speed'] = drone_speed_in_knots
        self.drone_info['speed_in_cm'] = GPSRAW_msg.vel

    def compass_hdg_callback(self, compass_hdg_msg):
        self.get_logger().info(f'received: \n{compass_hdg_msg.data}, {compass_hdg_msg.data}')
        self.drone_info['heading'] = compass_hdg_msg.data
        self.drone_info['track'] = compass_hdg_msg.data

    def publish_air_traffic_new_coordinates(self):
        msg = String()
        if self.air_traffic:
            for entry in self.air_traffic:
                self.air_traffic[entry]['new_coordinates'] = calculate_new_coordinates_using_nautical_miles(self.air_traffic[entry])

            msg.data = json.dumps(self.air_traffic, indent=1)
            self.air_traffic_new_coordinates_publisher.publish(msg)
            self.get_logger().info(f'Publishing air traffic with new coordinates:\n{msg.data}')
        else:
            self.get_logger().info(f'airspace empty')

    def publish_drone_new_coordinates(self):
        msg = String()
        if self.drone_info:
            self.drone_info['new_coordinates'] = calculate_new_coordinates_using_nautical_miles(self.drone_info)
            msg.data = json.dumps(self.drone_info, indent=1)
            self.drone_new_coordinates_publisher.publish(msg)
            self.get_logger().info(f'Publishing drone_info with new coordinates:\n{msg.data}')
        else:
            self.get_logger().info(f'drone info not available')
      


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
    # test_me()
    main()