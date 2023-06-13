from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
   TOPIC = "/air_traffic"

   formatted_data = Node(
      package='my_first_pkg',
      executable='talker',
      name='air_traffic_format_node',
      # parameters=[{
      #    "topic" : TOPIC
      # }]
   )

   new_coordinates = Node(
      package='my_first_pkg',
      executable='listener',
      name='new_coordinates_node',
      # parameters=[{
      #    "topic" : TOPIC
      # }]
   )

   advisor = Node(
      package='my_first_pkg',
      executable='collision_advisor',
      name='advisor_node',
      # parameters=[{
      #    "topic" : TOPIC
      # }]
   )

   return LaunchDescription([
      formatted_data,
      new_coordinates,
      advisor
   ])