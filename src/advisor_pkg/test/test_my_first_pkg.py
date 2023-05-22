import os
import sys
import time
import pytest
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy

import std_msgs.msg



def generate_test_description():
    file_path = os.path.dirname(__file__)
    air_traffic_format_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, '..', 'my_first_pkg', 'publisher.py')],
        additional_env= {'PYTHONUNBUFFED': '1'},
    )

    new_coordinates_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, '..', 'my_first_pkg', 'subscriber.py')],
        additional_env= {'PYTHONUNBUFFED': '1'},
    )

    advisor_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, '..', 'my_first_pkg', 'collision_advisor.py')],
        additional_env= {'PYTHONUNBUFFED': '1'},
    )

    return (
        launch.LaunchDescription([
            air_traffic_format_node,
            new_coordinates_node,
            advisor_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {

        }
    )

class TestThis(unittest.TestCase):

    @classmethod'
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def
    
