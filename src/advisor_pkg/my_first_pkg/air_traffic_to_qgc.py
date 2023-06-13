import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mavros_msgs.msg import ADSBVehicle

import json

import sys

sys.path.append('../advisor_pkg')

from my_first_pkg.format_data_example import data_to_publish
from my_first_pkg.aircraft_json_post_example import get_data, post_id_token, post_url

# from .submodules.format_data_example import data_to_publish
# from .aircraft_json_post_example import get_data, post_id_token, post_url


class AirTrafficPublisher(Node):

    def __init__(self):
        super().__init__('pub_to_qgc_node')

        self.pub_air_traffic = self.create_publisher(ADSBVehicle, '/mavros/adsb/send', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        # Subscriber to get current air traffic over Fyn
        self.air_traffic_subscription = self.create_subscription(
            String,
            '/air_traffic',
            self.air_traffic_callback,
            10
        )
        self.air_traffic_subscription  # prevent unused variable warning



        self.air_traffic = {}

    def air_traffic_callback(self, air_traffic_msg):
        # self.get_logger().info(f'received: \n{air_traffic_msg.data}')
        self.air_traffic = json.loads(air_traffic_msg.data)

        

    def timer_callback(self):

        msg = ADSBVehicle()

        for aircraft in self.air_traffic:
            aircraft_info = self.air_traffic[aircraft]
            msg.callsign = str(aircraft)
            print(aircraft_info)
            msg.icao_address = int(aircraft_info['icao_id'], base=16)

            msg.latitude = float(aircraft_info['lat'])
            msg.longitude = float(aircraft_info['lon'])

            if aircraft_info.get('baro_alt') == None:
                msg.altitude = float(aircraft_info['alt'])
            else:
                msg.altitude = float(aircraft_info['baro_alt'])

            msg.heading = float(aircraft_info['track'])

            msg.hor_velocity = float(aircraft_info['speed']) # float
            # msg.ver_velocity = float(data.data.vSpeed) # float
            
            msg.altitude_type = 1 # int
            msg.emitter_type = 1 # int

            msg.tslc.sec = 1 # int
            msg.tslc.nanosec = 0

            msg.flags = 31
            msg.squawk = 0

            # if msg.latitude != 0.0 and msg.longitude != 0.0 and msg.altitude < int(NAVIAIR_FILTER_ALT):
            self.pub_air_traffic.publish(msg)


        # msg.data = data_to_publish(json_string)
        # self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing airtraffic: \n{msg.data}')


def main(args=None):
    rclpy.init(args=args)

    air_traffic_publisher = AirTrafficPublisher()

    rclpy.spin(air_traffic_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    air_traffic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()