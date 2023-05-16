from ..my_first_pkg.calculate_collision_risk import calc_mission_collision
import pytest

def test_calc_mission_collision_with_collision():
    mission_waypoints = [(10, 20, 30), (40, 50, 60), (70, 80, 90)]
    air_traffic_dict = {
        'A1': {
            'new_coordinates': {
                '60.0': (15, 25, 35)
            }
        },
        'A2': {
            'new_coordinates': {
                '60.0': (45, 55, 65)
            }
        },
        'A3': {
            'new_coordinates': {
                '60.0': (75, 85, 95)
            }
        }
    }

    result = calc_mission_collision(mission_waypoints, air_traffic_dict)
    expected = ['warn', 'Careful! A1,A2,A3 will collide with your mission within the next 60 seconds!']
    assert result == expected

def test_calc_mission_collision_without_collision():
    mission_waypoints = [(10, 20, 30), (40, 50, 60), (70, 80, 90)]
    air_traffic_dict = {
        'A1': {
            'new_coordinates': {
                '60.0': (12, 22, 32)
            }
        },
        'A2': {
            'new_coordinates': {
                '60.0': (45, 55, 65)
            }
        },
        'A3': {
            'new_coordinates': {
                '60.0': (75, 85, 95)
            }
        }
    }

    result = calc_mission_collision(mission_waypoints, air_traffic_dict)
    expected = ['info', 'No Aircraft(s) will collide with the mission waypoints']
    assert result == expected

