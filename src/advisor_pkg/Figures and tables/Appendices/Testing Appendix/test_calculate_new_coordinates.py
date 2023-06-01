import sys

sys.path.append('../advisor_pkg')
from my_first_pkg.calculate_new_coordinates import calculate_new_coordinates_using_nautical_miles

import pytest

def test_valid_coordinates():
    aircraft = {'lat': 51.5074, 'lon': -0.1278, 'alt': 10000, 'track': 90, 'speed': 400}
    time = 60.0
    expected_result = {time: [51.507265006402875, 0.050396192133436685, 10000]}
    result = calculate_new_coordinates_using_nautical_miles(aircraft, time)
    assert result == expected_result

def test_missing_baro_altitude():
    aircraft = {'lat': 55.5074, 'lon': -10.1278, 'alt': 200, 'track': 10, 'speed': 500}
    time = 60.0
    expected_result = {time: [55.64392628749115, -10.085139910355801, 200]}
    result = calculate_new_coordinates_using_nautical_miles(aircraft, time)
    assert result == expected_result

def test_missing_alt_altitude():
    aircraft = {'lat': 80.5018, 'lon': -90.1278, 'baro_alt': 5000, 'track': 65, 'speed': 200}
    time = 60.0
    expected_result = {time: [80.52510462484614, -89.82247985980798, 5000]}
    result = calculate_new_coordinates_using_nautical_miles(aircraft, time)
    assert result == expected_result

def test_zero_speed():
    aircraft = {'lat': 51.5074, 'lon': -0.1278, 'alt': 10000, 'track': 90, 'speed': 0}
    time = 60.0
    expected_result = {time: [51.5074, -0.1278, 10000]}
    result = calculate_new_coordinates_using_nautical_miles(aircraft, time)
    assert result == expected_result