# import os
# import sys
# import time
# import pytest
# import unittest
# import uuid

# import launch
# import launch_ros
# import launch_ros.actions
# import launch_testing.actions

# import rclpy

# import std_msgs.msg


# @pytest.mark.rostest
# def generate_test_description():
#     file_path = os.path.dirname(__file__)
#     air_traffic_format_node = launch_ros.actions.Node(
#         executable=sys.executable,
#         arguments=[os.path.join(
#             os.path.abspath(os.path.join(file_path, os.pardir)), "my_first_pkg", "publisher.py")],
#         additional_env= {'PYTHONUNBUFFED': '1'},
#         parameters=[{
#             'air_traffic_pub_param' : 'air_traffic_talker'
#         }]
#     )

#     new_coordinates_node = launch_ros.actions.Node(
#         executable=sys.executable,
#         arguments=[os.path.join(
#             os.path.abspath(os.path.join(file_path, os.pardir)), "my_first_pkg", "subscriber.py")],
#         additional_env= {'PYTHONUNBUFFED': '1'},
#         parameters=[{
#             'air_traffic_sub_param' : 'air_traffic_listener',
#             'air_traffic_new_pub_param' : 'air_traffic_new_talker',
#             'drone_new_pub_param': 'drone_new_talker'
#         }]
#     )

#     advisor_node = launch_ros.actions.Node(
#         executable=sys.executable,
#         arguments=[os.path.join(
#             file_path, '..', 'my_first_pkg', 'collision_advisor.py')],
#         additional_env= {'PYTHONUNBUFFED': '1'},
#         parameters=[{
#             'publish_topic' : 'advisor_talker',
#             'air_traffic_new_sub_param' : 'asd' # TODO - DO NOT WORK 
#         }]
#     )

#     return (
#         launch.LaunchDescription([
#             air_traffic_format_node,
#             new_coordinates_node,
#             advisor_node,
#             launch_testing.actions.ReadyToTest(),
#         ]),
#         {
#             'air_traffic_talker_node' : air_traffic_format_node,
#             'new_coordinates_node' : new_coordinates_node,
#             # 'new_coordinates_talker_node' : new_coordinates_node,
#             'advisor_listener' : advisor_node
#         }
#     )

# class TestAdvisor_pkgLink(unittest.TestCase):

#     @classmethod
#     def setUpClass(cls) -> None:
#         rclpy.init()
    
#     @classmethod
#     def tearDownClass(cls) -> None:
#         rclpy.shutdown()

#     def setUp(self) -> None:
#         self.node = rclpy.create_node('test_my_first_pkg_link')

#     def tearDown(self) -> None:
#         self.node.destroy_node()
    
#     def test_air_traffic_transmits(self, air_traffic_talker_node, proc_output):
#         msgs_rx = []

#         sub = self.node.create_subscription(
#             std_msgs.msg.String,
#             'air_traffic_talker',
#             lambda msg: msgs_rx.append(msg),
#             10
#         )
    
#         try:
#             end_time = time.time() + 10
#             while time.time() < end_time:
#                 rclpy.spin_once(self.node, timeout_sec=0.1)
#                 if len(msgs_rx) > 2:
#                     break
            
#             self.assertGreater(len(msgs_rx), 2)
            

#             for msg in msgs_rx:

#                 print('this is msg', msg)
#                 proc_output.assertWaitFor(
#                     expected_output = msg.data,
#                     process = air_traffic_talker_node
#                 )

#         finally:
#             self.node.destroy_subscription(sub)

#     def test_new_coordinates_receives(self, new_coordinates_node, proc_output):
#         pub = self.node.create_publisher(
#             std_msgs.msg.String,
#             'air_traffic_listener',
#             10
#         )

#         try:
#             msg = std_msgs.msg.String()
#             msg.data = str(uuid.uuid4())
#             for _ in range(10):
#                 pub.publish(msg)
#                 success = proc_output.waitFor(
#                     expected_output = msg.data,
#                     process = new_coordinates_node,
#                     timeout = 0.1
#                 )
#                 if success:
#                     break
#             assert success, 'Waiting for output timed out'
#         finally:
#             self.node.destroy_publisher(pub)
        
#     def test_new_coordinates_transmits(self, new_coordinates_node, proc_output):
#         msgs_rx = []

#         sub = self.node.create_subscription(
#             std_msgs.msg.String,
#             'air_traffic_new_talker',
#             lambda msg: msgs_rx.append(msg),
#             10
#         )

#         try:
#             end_time = time.time() + 10
#             while time.time() < end_time:
#                 rclpy.spin_once(self.node, timeout_sec=0.1)
#                 if len(msgs_rx) > 2:
#                     break
            
#             self.assertGreater(len(msgs_rx), 2)
            
#             for msg in msgs_rx:
#                 print('this is msg', msg)
#                 proc_output.assertWaitFor(
#                     expected_output = msg.data,
#                     process = new_coordinates_node
#                 )

#         finally:
#             self.node.destroy_subscription(sub)

#     def test_advisor_receives(self, advisor_listener, proc_output):
#         pub = self.node.create_publisher(
#             std_msgs.msg.String,
#             'advisor_chatter',
#             10
#         )

#         try:
#             msg = std_msgs.msg.String()
#             msg.data = str(uuid.uuid4())
#             for _ in range(10):
#                 pub.publish(msg)
#                 success = proc_output.waitFor(
#                     expected_output = msg.data,
#                     process = advisor_listener,
#                     timeout = 0.1
#                 )
#                 if success:
#                     break
#             assert success, 'Waiting for output timed out'
#         finally:
#             self.node.destroy_publisher(pub)