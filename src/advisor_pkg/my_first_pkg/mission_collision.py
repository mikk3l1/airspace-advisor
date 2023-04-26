import rclpy
from rclpy.node import Node

import json

from std_msgs.msg import String
from mavros_msgs.msg import WaypointList



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('mission_subscriber')

        # Subscriber to get the waypoints of the drones mission
        self.subscription = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_mission_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Subscriber to get current air traffic over Fyn
        self.air_traffic_subscription = self.create_subscription(
            String,
            'air_traffic',
            self.air_traffic_callback,
            10
        )
        self.air_traffic_subscription  # prevent unused variable warning


        self.publisher = self.create_publisher(String, '/mission_advise', 10)

        # Timer to publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_msg)


    def waypoints_mission_callback(self, waypoints_msg):
        for wayPoint in waypoints_msg.waypoints:
            self.get_logger().info(f'I heard:\nlat:{wayPoint.x_lat}\nlong:{wayPoint.y_long}')


    def air_traffic_callback(self, air_traffic_msg):
        self.air_traffic = json.loads(air_traffic_msg.data)    

    # TODO: Skal kunne fortælle om drone missions 'waypoints' kollidere med et fly given ud fra en tid
    def publish_msg(self):
        # msg = String()
        # msg.data = get_advise(self.air_traffic, self.drone_info)
        # self.publisher.publish(msg)
        # self.get_logger().info(f'Publishing message to new_topic: \n{msg.data}' )
        pass


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