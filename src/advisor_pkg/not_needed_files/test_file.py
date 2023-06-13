'''
Only for testing purposes
Simple python scripts and other stuff

Should be deleted
'''


import geopy.distance
import os
import sys
import time
from math import radians, sin, cos, sqrt


# Geopy calculate the distance in kilometers between two objects on a sphere e.g. Earth
def distance_between_objects(object1: list, object2: list) -> float:
    return geopy.distance.great_circle(object1, object2).km


if __name__ == "__main__":
    drone_dict = {
        "lat": 55.4715442,
        "lon": 10.3231948,
        "alt": 97.19167246477875,
        "heading": 248.62,
        "track": 248.62,
        "speed": 9.50616,
        "speed_in_cm": 489,
        "new_coordinates": {
            "10": [
                55.47138404623427,
                10.322473088148882,
                97.19167246477875
            ],
            "30": [
                55.47106372596751,
                10.321029682039507,
                97.19167246477875
            ],
            "60": [
                55.47058321373006,
                10.318864616858692,
                97.19167246477875
            ],
            "120": [
                55.469622074646736,
                10.314534644843937,
                97.19167246477875
            ],
            "180": [
                55.46866078276296,
                10.310204883971421,
                97.19167246477875
            ],
            "300": [
                55.46673774064588,
                10.301545995715813,
                97.19167246477875
            ]
        }
    }

    air_traffic_dict1 = {
        "DOC93": {
            "seen": -3.991,
            "source_type": "ADSB",
            "source_name": "Unknown",
            "type": "MED_HELI",
            "icao_id": "47A1CE",
            "flight": "DOC93",
            "lat": "55.45214",
            "lon": "10.31778",
            "gnss_alt": 252,
            "baro_alt": 899,
            "track": 8,
            "speed": 97,
            "sim": "true",
            "target_dist": 2642,
            "target_dir": 193,
            "new_coordinates": {
                "10": [
                    55.456579060909256,
                    10.318880240422237,
                    899.0
                ],
                "30": [
                    55.46545715311348,
                    10.32108146441044,
                    899.0
                ],
                "60": [
                    55.47877421732602,
                    10.32438515969745,
                    899.0
                ],
                "120": [
                    55.50540807863059,
                    10.330999253322453,
                    899.0
                ],
                "180": [
                    55.53204158307678,
                    10.337622301757163,
                    899.0
                ],
                "300": [
                    55.585307518033424,
                    10.350895346902169,
                    899.0
                ]
            }
        },
        "NSZ3502": {
            "seen": 1.044,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "4ACB23",
            "flight": "NSZ3502",
            "lat": "55.82794",
            "lon": "9.64674",
            "gnss_alt": 0,
            "baro_alt": 32723,
            "track": 273,
            "speed": 462,
            "sim": "false",
            "target_dist": 57949,
            "target_dir": 313,
            "new_coordinates": {
                "10": [
                    55.8290515576669,
                    9.608779061586063,
                    32723.0
                ],
                "30": [
                    55.83123960707218,
                    9.53285073234216,
                    32723.0
                ],
                "60": [
                    55.83443399937997,
                    9.41894244668071,
                    32723.0
                ],
                "120": [
                    55.84050702102765,
                    9.19107126647263,
                    32723.0
                ],
                "180": [
                    55.84615883735075,
                    8.963131356368912,
                    32723.0
                ],
                "300": [
                    55.85619802189435,
                    8.507064985130864,
                    32723.0
                ]
            }
        },
        "NSZ3516": {
            "seen": 1.044,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "4ACB03",
            "flight": "NSZ3516",
            "lat": "54.91333",
            "lon": "10.82751",
            "gnss_alt": 0,
            "baro_alt": 29600,
            "track": 238,
            "speed": 491,
            "sim": "false",
            "target_dist": 70105,
            "target_dir": 153,
            "new_coordinates": {
                "10": [
                    54.9013011438175,
                    10.794043423512594,
                    29600.0
                ],
                "30": [
                    54.877215869624486,
                    10.727170255383284,
                    29600.0
                ],
                "60": [
                    54.841019197577346,
                    10.62701038040563,
                    29600.0
                ],
                "120": [
                    54.768379262190145,
                    10.427229617744338,
                    29600.0
                ],
                "180": [
                    54.695412256531796,
                    10.228166440842456,
                    29600.0
                ],
                "300": [
                    54.548505246668604,
                    9.832187540392301,
                    29600.0
                ]
            }
        }
    }

    air_traffic_dict2 = {
        "AFR94XK": {
            "seen": 1,
            "source_type": "ADSB",
            "source_name": "OUH2",
            "icao_id": "39E680",
            "flight": "AFR94XK",
            "lat": "55.11067",
            "lon": "9.70464",
            "gnss_alt": 0,
            "baro_alt": 36998,
            "track": 46,
            "speed": 412,
            "sim": "false",
            "target_dist": 2642,
            "target_dir": 193,
            "new_coordinates": {
                "10": [
                    55.12389386624683,
                    9.728592527738117,
                    36998.0
                ],
                "30": [
                    55.15032749209805,
                    9.776545192773785,
                    36998.0
                ],
                "60": [
                    55.18994258275856,
                    9.84859340430635,
                    36998.0
                ],
                "120": [
                    55.26904497103097,
                    9.993120252661628,
                    36998.0
                ],
                "180": [
                    55.34797598225237,
                    10.138223284173508,
                    36998.0
                ],
                "300": [
                    55.50531910238444,
                    10.43016886038301,
                    36998.0
                ]
            }
        },
        "DOC93": {
            "seen": -3.991,
            "source_type": "ADSB",
            "source_name": "Unknown",
            "type": "MED_HELI",
            "icao_id": "47A1CE",
            "flight": "DOC93",
            "lat": "55.45214",
            "lon": "10.31778",
            "gnss_alt": 252,
            "baro_alt": 899,
            "track": 8,
            "speed": 97,
            "sim": "true",
            "target_dist": 2642,
            "target_dir": 193,
            "new_coordinates": {
                "10": [
                    55.456579060909256,
                    10.318880240422237,
                    899.0
                ],
                "30": [
                    55.46545715311348,
                    10.32108146441044,
                    899.0
                ],
                "60": [
                    55.47877421732602,
                    10.32438515969745,
                    899.0
                ],
                "120": [
                    55.50540807863059,
                    10.330999253322453,
                    899.0
                ],
                "180": [
                    55.53204158307678,
                    10.337622301757163,
                    899.0
                ],
                "300": [
                    55.585307518033424,
                    10.350895346902169,
                    899.0
                ]
            }
        },
        "NSZ3502": {
            "seen": 1.044,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "4ACB23",
            "flight": "NSZ3502",
            "lat": "55.82794",
            "lon": "9.64674",
            "gnss_alt": 0,
            "baro_alt": 32723,
            "track": 273,
            "speed": 462,
            "sim": "false",
            "target_dist": 57949,
            "target_dir": 313,
            "new_coordinates": {
                "10": [
                    55.8290515576669,
                    9.608779061586063,
                    32723.0
                ],
                "30": [
                    55.83123960707218,
                    9.53285073234216,
                    32723.0
                ],
                "60": [
                    55.83443399937997,
                    9.41894244668071,
                    32723.0
                ],
                "120": [
                    55.84050702102765,
                    9.19107126647263,
                    32723.0
                ],
                "180": [
                    55.84615883735075,
                    8.963131356368912,
                    32723.0
                ],
                "300": [
                    55.85619802189435,
                    8.507064985130864,
                    32723.0
                ]
            }
        }
    }

    air_traffic_dict = {
        "AFR94XK": {
            "seen": 1,
            "source_type": "ADSB",
            "source_name": "OUH2",
            "icao_id": "39E680",
            "flight": "AFR94XK",
            "lat": "55.12389386624683",
            "lon": "9.728592527738117",
            "gnss_alt": 0,
            "baro_alt": 36998,
            "track": 46,
            "speed": 412,
            "sim": "false",
            "target_dist": 56535,
            "target_dir": 224,
            "new_coordinates": {
                "10": [
                    55.12389386624683,
                    9.728592527738117,
                    36998.0
                ],
                "30": [
                    55.15032749209805,
                    9.776545192773785,
                    36998.0
                ],
                "60": [
                    55.18994258275856,
                    9.84859340430635,
                    36998.0
                ],
                "120": [
                    55.26904497103097,
                    9.993120252661628,
                    36998.0
                ],
                "180": [
                    55.34797598225237,
                    10.138223284173508,
                    36998.0
                ],
                "300": [
                    55.50531910238444,
                    10.43016886038301,
                    36998.0
                ]
            }
        },
        "DOC93": {
            "seen": -3.991,
            "source_type": "ADSB",
            "source_name": "Unknown",
            "type": "MED_HELI",
            "icao_id": "47A1CE",
            "flight": "DOC93",
            "lat": "55.456579060909256",
            "lon": "10.318880240422237",
            "gnss_alt": 252,
            "baro_alt": 899,
            "track": 8,
            "speed": 97,
            "sim": "true",
            "target_dist": 2642,
            "target_dir": 193,
            "new_coordinates": {
                "10": [
                    57.456579060909256,
                    10.318880240422237,
                    899.0
                ],
                "30": [
                    57.46545715311348,
                    10.32108146441044,
                    899.0
                ],
                "60": [
                    57.47877421732602,
                    10.32438515969745,
                    899.0
                ],
                "120": [
                    54.50540807863059,
                    10.330999253322453,
                    899.0
                ],
                "180": [
                    53.53204158307678,
                    10.337622301757163,
                    899.0
                ],
                "300": [
                    55.53204158307678,
                    10.337622301757163,
                    899.0
                ]
            }
        },
        "NSZ3502": {
            "seen": 1.044,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "4ACB23",
            "flight": "NSZ3502",
            "lat": "55.8290515576669",
            "lon": "9.608779061586063",
            "gnss_alt": 0,
            "baro_alt": 32723,
            "track": 273,
            "speed": 462,
            "sim": "false",
            "target_dist": 57949,
            "target_dir": 313,
            "new_coordinates": {
                "10": [
                    55.53204158307678,
                    10.337622301757163,
                    899.0
                ],
                "30": [
                    55.83123960707218,
                    9.53285073234216,
                    32723.0
                ],
                "60": [
                    55.83443399937997,
                    9.41894244668071,
                    32723.0
                ],
                "120": [
                    55.84050702102765,
                    9.19107126647263,
                    32723.0
                ],
                "180": [
                    55.84615883735075,
                    8.963131356368912,
                    32723.0
                ],
                "300": [
                    55.85619802189435,
                    8.507064985130864,
                    32723.0
                ]
            }
        },
        "NSZ3516": {
            "seen": 1.044,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "4ACB03",
            "flight": "NSZ3516",
            "lat": "54.9013011438175",
            "lon": "10.794043423512594",
            "gnss_alt": 0,
            "baro_alt": 29600,
            "track": 238,
            "speed": 491,
            "sim": "false",
            "target_dist": 70105,
            "target_dir": 173,
            "new_coordinates": {
                "10": [
                    54.9013011438175,
                    10.794043423512594,
                    29600.0
                ],
                "30": [
                    55.53204158307678,
                    10.337622301757163,
                    899.0
                ],
                "60": [
                    54.94481279189382,
                    10.626447719107978,
                    29600.0
                ],
                "120": [
                    55.00133167772439,
                    10.410894324485142,
                    29600.0
                ],
                "180": [
                    55.05730376328125,
                    10.195431033055847,
                    29600.0
                ],
                "300": [
                    55.53204158307678,
                    10.337622301757163,
                    899.0
                ]
            }
        }
    }

def calc_collision_time_drone_aircraft(drone_dict: dict, air_traffic_dict: dict):
    colliding_aircraft = dict()
    # flag = False
    for time, drone_coord in drone_dict['new_coordinates'].items():
        colliding_aircraft.setdefault(time, [])
        
        drone_lat, drone_lon, _ = drone_coord
        drone_lat = float(drone_lat)
        drone_lon = float(drone_lon)

        for flight, air_traffic_info in air_traffic_dict.items():
            flag = False  

            for lst in colliding_aircraft.values():
                if flight in lst:
                    flag = True
                    continue
            if flag:
                continue

            air_traffic_lat = float(
                air_traffic_info['new_coordinates'][time][0])  # Get lat
            air_traffic_lon = float(
                air_traffic_info['new_coordinates'][time][1])  # Get lon

            # Calculate the distance between the coordinates
            distance = distance_between_objects(
                (drone_lat, drone_lon), (air_traffic_lat, air_traffic_lon))

            # print(flight, distance, time)
            # Check if the distance is below a certain threshold (e.g., 1 kilometer)
            threshold = 10.0  # Adjust this value as needed
            if distance < threshold:
                colliding_aircraft[time].append(flight)
                # print(
                #     f"Aircraft {flight} may collide with the drone at time {time}.")
                # break  # Exit the loop after the first collision is detected
    return colliding_aircraft

print(calc_collision_time_drone_aircraft(drone_dict, air_traffic_dict), 'thjis is printed')