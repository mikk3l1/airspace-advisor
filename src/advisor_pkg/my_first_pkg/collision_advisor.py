import rclpy
from rclpy.node import Node

import json

from std_msgs.msg import String
from mavros_msgs.msg import WaypointList

import sys

sys.path.append('../advisor_pkg')

from my_first_pkg.calculate_collision_risk import calc_mission_collision
from my_first_pkg.calculate_collision_risk import calc_drone_aircraft_collision

# from .calculate_collision_risk import calc_mission_collision
# from .calculate_collision_risk import calc_drone_aircraft_collision

from my_first_pkg.advisor_log_text_req import get_data, post_test, post_url

# from .advisor_log_text_req import get_data
# from .advisor_log_text_req import post_test
# from .advisor_log_text_req import post_url



class CollisionSubscriber(Node):

    def __init__(self):
        super().__init__('advisor_node')


        self.declare_parameter('air_traffic_new_sub_param', value='/air_traffic_new_coordinates')
        air_traffic_new_sub_name = self.get_parameter('air_traffic_new_sub_param').get_parameter_value().string_value

        self.declare_parameter('drone_new_sub_param', value='/drone_new_coordinates')
        sub_drone_new_name = self.get_parameter('drone_new_sub_param').get_parameter_value().string_value

        self.declare_parameter('publish_topic', value='/advisor_topic')
        publish_advisor_name = self.get_parameter('publish_topic').get_parameter_value().string_value

        
        # Subscriber to get the drones mission waypoints
        self.WaypointList_subscription = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.WaypointList_callback,
            10)
        self.WaypointList_subscription  # prevent unused variable warning

        # Subscriber to get current air_traffic dict over Fyn with new_coordinates
        self.air_traffic_subscription = self.create_subscription(
            String,
            # '/air_traffic_new_coordinates',
            air_traffic_new_sub_name,
            self.air_traffic_new_coordinates_callback,
            10
        )
        self.air_traffic_subscription  # prevent unused variable warning


        # Subscriber to get drones dict with new_coordinates if it just fly straight
        self.drone_new_subscription = self.create_subscription(
            String,
            # 'drone_new_coordinates',
            sub_drone_new_name,
            self.drone_new_callback,
            10
        )
        self.drone_new_subscription  # prevent unused variable warning

        # Creating a publisher
        # self.advisor_publisher = self.create_publisher(String, '/advisor_topic', 10)
        self.advisor_publisher = self.create_publisher(String, publish_advisor_name, 10)

        # Timer to publish every 2 seconds
        self.advisor_timer = self.create_timer(2.0, self.publish_to_advisor_thread)


        self.drone_dict = {}
        self.air_traffic_dict = {}
        self.waypoints_list = []

    def WaypointList_callback(self, waypoints_msg):
        self.get_logger().info(f'WaypointList received: \n{waypoints_msg}')
        self.waypoints_list = []
        for wayPoint in waypoints_msg.waypoints:
            self.waypoints_list.append([wayPoint.x_lat,wayPoint.y_long,wayPoint.z_alt])
            # self.get_logger().info(f'I heard:\nlat:{wayPoint.x_lat}\nlong:{wayPoint.y_long}\nalt:{wayPoint.z_alt}')
        
    def air_traffic_new_coordinates_callback(self, air_traffic_new_coordinates_msg):
        self.get_logger().info(f'air_traffic_new_coordinates received: \n{air_traffic_new_coordinates_msg.data}')
        self.air_traffic_dict = json.loads(air_traffic_new_coordinates_msg.data)
        # self.get_logger().info(f'I heard:\n {json.dumps(self.air_traffic_dict, indent=1)}')

    def drone_new_callback(self, drone_new_coordinates_msg):
        self.get_logger().info(f'drone_new received: \n{drone_new_coordinates_msg.data}')
        self.drone_dict = json.loads(drone_new_coordinates_msg.data)
        # self.get_logger().info(f'I heard:\n {json.dumps(self.drone_dict, indent=1)}')


    def publish_to_advisor_thread(self):
        msg = String()

        if self.air_traffic_dict and self.waypoints_list:
            mission_collision = calc_mission_collision(self.waypoints_list, self.air_traffic_dict)
            post_test['level'] = mission_collision[0]
            post_test['text'] = mission_collision[1]
            self.get_logger().info(f'Mission:\nPOST to {post_url} with {post_test}')
            get_data(post_url, post_test)

        else:
            if not self.air_traffic_dict:
                self.get_logger().info(f'air_traffic_dict is empty')
            else:
                self.get_logger().info(f'self.waypoints_list is empty')

        if self.air_traffic_dict and self.drone_dict and self.drone_dict['speed'] > 1:
            aircraft_collision = calc_drone_aircraft_collision(self.drone_dict,self.air_traffic_dict)
            post_test['level'] = aircraft_collision[0]
            post_test['text'] = aircraft_collision[1]
            self.get_logger().info(f'Drone-Aircraft:\nPOST to {post_url} with {post_test}')
            get_data(post_url, post_test)
        else:
            if not self.air_traffic_dict:
                self.get_logger().info(f'air_traffic_dict is empty')
            else:
                self.get_logger().info(f'self.drone_dict is empty or drone is flying too slow')



def main(args=None):
    rclpy.init(args=args)

    collision_subscriber = CollisionSubscriber()

    rclpy.spin(collision_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    collision_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()