import sys

sys.path.append('../advisor_pkg')
from my_first_pkg.calculate_collision_risk import calc_mission_collision
from my_first_pkg.calculate_collision_risk import calc_drone_aircraft_collision

import pytest

def test_calc_mission_collision_with_collision():
    mission_waypoints = [(40.7128, -74.0060, 30.0), (34.0522, -118.2437, 60.0), (51.5074, -0.1278, 90.0)]
    air_traffic_dict = {
        'A1': {
            'new_coordinates': {
                '60.0': [40.7129, -74.0061, 35.0]
            }
        },
        'A2': {
            'new_coordinates': {
                '60.0': [34.0523, -118.2438, 65.0]
            }
        },
        'A3': {
            'new_coordinates': {
                '60.0': [51.5075, -0.1279, 95.0]
            }
        }
    }

    result = calc_mission_collision(mission_waypoints, air_traffic_dict)
    expected = ['warn', 'Careful! A2,A3, will collide with your mission within the next 60 seconds!']
    assert result == expected

def test_calc_mission_collision_without_collision():
    mission_waypoints = [(40.7128, -74.0060, 30.0), (34.0522, -118.2437, 30.0), (51.5074, -0.1278, 30.0)]
    air_traffic_dict = {
        'A1': {
            'new_coordinates': {
                '60.0': [42.7129, -79.0061, 60.0]
            }
        },
        'A2': {
            'new_coordinates': {
                '60.0': [30.0523, -100.2438, 65.0]
            }
        },
        'A3': {
            'new_coordinates': {
                '60.0': [58.5075, -10.1279, 95.0]
            }
        }
    }

    result = calc_mission_collision(mission_waypoints, air_traffic_dict)
    expected = ['info', 'No Aircraft(s) will collide with the mission waypoints']
    assert result == expected


def test_calc_drone_aircraft_collision_with_collision():
    drone_dict = {
        'new_coordinates': {
            '60.0': [40.7128, -74.0060, 30.0]
        }
    }

    air_traffic_dict = {
        'A1': {
            'new_coordinates': {
                '60.0': [40.7129, -74.0061, 35.0]
            }
        },
        'A2': {
            'new_coordinates': {
                '60.0': [51.5075, -0.1279, 95.0]
            }
        },
        'A3': {
            'new_coordinates': {
                '60.0': [40.7199, -74.0061, 65.0]
            }
        }
    }

    result = calc_drone_aircraft_collision(drone_dict, air_traffic_dict)
    expected = ['warn', 'Collision with A1,A3, will happen in the next 60 seconds!']
    assert result == expected

def test_calc_drone_aircraft_collision_without_collision():
    drone_dict = {
        'new_coordinates': {
            '60.0': [40.7128, -74.0060, 30.0]
        }
    }

    air_traffic_dict = {
        'A1': {
            'new_coordinates': {
                '60.0': [45.7129, -74.0061, 32.0]
            }
        },
        'A2': {
            'new_coordinates': {
                '60.0': [34.0523, -118.2438, 65.0]
            }
        },
        'A3': {
            'new_coordinates': {
                '60.0': [51.5075, -0.1279, 95.0]
            }
        }
    }

    result = calc_drone_aircraft_collision(drone_dict, air_traffic_dict)
    expected = ['info', 'No Aircraft will collide with the drone in the next 60 seconds']
    assert result == expected