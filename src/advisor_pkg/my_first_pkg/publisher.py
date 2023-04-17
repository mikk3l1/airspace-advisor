import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json
from .format_data_example import data_to_publish
from .aircraft_json_post_example import get_data, post_id_token, post_url


class AirTrafficPublisher(Node):

    def __init__(self):
        super().__init__('air_traffic_publisher')
        self.publisher_ = self.create_publisher(String, 'air_traffic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        json_object = json.loads(get_data(post_url, post_id_token))
        json_string = json.dumps(json_object, indent=1)

        msg = String()
        msg.data = data_to_publish(json_string)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: \n{msg.data}')      


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