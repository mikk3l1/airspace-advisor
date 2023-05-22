import sys

sys.path.append('../advisor_pkg')
from my_first_pkg.format_data_example import data_to_publish

import pytest

# def test_valid_json_str():
#     json_str = '{"aircraft": [{"flight": "ABC123", "lat": 1.234, "lon": 5.678, "track": 90, "speed": 300}]}'
#     expected_result = '{"ABC123": {"flight": "ABC123", "lat": 1.234, "lon": 5.678, "track": 90, "speed": 300}}'
#     result = data_to_publish(json_str)
#     assert result == expected_result

def test_invalid_json_str():
    json_str = '{"aircraft": [{"flight": "ABC123", "lat": 1.234}]}'
    expected_result = '{}'
    result = data_to_publish(json_str)
    assert result == expected_result

def test_none_flight_value():
    json_str = '{"aircraft": [{"flight": null, "lat": 1.234, "lon": 5.678, "track": 90, "speed": 300}]}'
    expected_result = '{}'
    result = data_to_publish(json_str)
    assert result == expected_result