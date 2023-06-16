import geopy.distance
import json


# Geopy calculate the distance in kilometers between two objects on a sphere e.g. Earth
def distance_between_objects(object1: list, object2: list) -> float:
    return geopy.distance.great_circle(object1, object2).km


def calc_mission_collision(mission_waypoints: list, air_traffic_dict: dict) -> list:
    waypoint_aircraft_distance_dict = {}
    waypoint_aircraft_collision = ''

    for aircraft in air_traffic_dict:
        aircraft_new_coordinates_list = air_traffic_dict[aircraft]['new_coordinates'].get(
            '60.0')

        for index, waypoint in enumerate(mission_waypoints[0:len(mission_waypoints)-1]):
            waypoint_aircraft_distance_dict[aircraft] = distance_between_objects(
                waypoint[0:2], aircraft_new_coordinates_list[0:2])

        # FFA well-clear
        # 2_000 ft horizontal (0.6096 km)
        # 600 ft vertical (182.88 meter)
        if any(i < 1 for i in waypoint_aircraft_distance_dict.values()) and abs(waypoint[2]-aircraft_new_coordinates_list[2]) < 200:
            waypoint_aircraft_collision += f'{aircraft},'

    if waypoint_aircraft_collision:
        return ['warn', f'Careful! {waypoint_aircraft_collision} will collide with your mission within the next 60 seconds!']
    else:
        return ['info', f'No Aircraft will collide with the mission waypoints']


def calc_drone_aircraft_collision(drone_dict: dict, air_traffic_dict: dict) -> list:
    drone_aircraft_collision = ''
    drone_new_coordinates_lat_long = drone_dict['new_coordinates'].get('60.0')

    for aircraft in air_traffic_dict:
        aircraft_new_coordinates_list_lat_long = air_traffic_dict[aircraft]['new_coordinates'].get(
            '60.0')

        distance_between_drone_aircraft = distance_between_objects(drone_new_coordinates_lat_long[0:2], aircraft_new_coordinates_list_lat_long[0:2])
        
        altitude_between_drone_aircraft = abs(drone_new_coordinates_lat_long[2:3][0])-abs(aircraft_new_coordinates_list_lat_long[2:3][0])

        # FFA well-clear
        # 2_000 ft horizontal (0.6096 kilometer / 609,6 meter)
        # 600 ft vertical (0,18288 kilometer / 182.88 meter)
        if distance_between_drone_aircraft < 1 and altitude_between_drone_aircraft < 200:
            drone_aircraft_collision += f'{aircraft},'

    if drone_aircraft_collision:
        return ['warn', f'Collision with {drone_aircraft_collision} will happen in the next 60 seconds!']
    else:
        return ['info', f'No Aircraft will collide with the drone in the next 60 seconds']


# TODO
def calc_collision_time_mission_aircraft(mission_waypoints: list, air_traffic_dict: dict):
    collision_aircraft_mission = dict()
    
    for index, waypoint in enumerate(mission_waypoints):
        for flight, aircraft in air_traffic_dict.items():
            for time, aircraft_coords in aircraft['new_coordinates'].items():
                collision_aircraft_mission.setdefault(time, [])
                waypoint_lat, waypoint_lon, _ = waypoint
                aircraft_lat, aircraft_lon, _ = aircraft_coords

                flag = False  

                for lst in collision_aircraft_mission.values():
                    if flight in lst:
                        flag = True
                        continue
                if flag:
                    continue # Skip the loop if the flight is detected to be in "colliding_aircraft" dict

                distance = distance_between_objects((waypoint_lat, waypoint_lon), (aircraft_lat, aircraft_lon))
                
                # FFA well-clear
                # 2_000 ft horizontal (0.6096 km)
                # 600 ft vertical (182.88 meter)

                # Check if the distance is below a certain threshold (e.g., 1 kilometer)
                threshold = 1.0  # Adjust this value as needed
                if distance < threshold:
                    collision_aircraft_mission[time].append(flight)
                    # print(flight, time, index, waypoint)

    return collision_aircraft_mission

    # pass



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
                continue # Skip the loop if the flight is detected to be in "colliding_aircraft" dict

            air_traffic_lat = float(
                air_traffic_info['new_coordinates'][time][0])  # Get lat
            air_traffic_lon = float(
                air_traffic_info['new_coordinates'][time][1])  # Get lon

            # Calculate the distance between the coordinates
            distance = distance_between_objects(
                (drone_lat, drone_lon), (air_traffic_lat, air_traffic_lon))

            # FFA well-clear
            # 2_000 ft horizontal (0.6096 km)
            # 600 ft vertical (182.88 meter)

            # Check if the distance is below a certain threshold (e.g., 1 kilometer)
            threshold = 1.0  # Adjust this value as needed
            if distance < threshold:
                colliding_aircraft[time].append(flight)

    return colliding_aircraft


if __name__ == '__main__':

    drone_dict = {
        "lat": 55.4718504,
        "lon": 10.3245889,
        "alt": 47.41489522387912,
        "heading": 90.24,
        "track": 90.24,
        "speed": 0.07776,
        "speed_in_cm": 4,
        "new_coordinates": {
            "10": [
                55.47185038494726,
                10.324595239882363,
                47.41489522387912
            ],
            "30": [
                55.471850354840825,
                10.324607919647073,
                47.41489522387912
            ],
            "60": [
                55.47185030967868,
                10.324626939294102,
                47.41489522387912
            ],
            "120": [
                55.47185021934556,
                10.324664978588027,
                47.41489522387912
            ],
            "180": [
                55.471850129000664,
                10.324703017881781,
                47.41489522387912
            ],
            "300": [
                55.471849948275484,
                10.324779096468761,
                47.41489522387912
            ]
        }
    }

    air_traffic_dict = {
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
        "N2439R": {
            "seen": 20,
            "source_type": "ADSB",
            "source_name": "Unknown",
            "icao_id": "A23EC7",
            "flight": "N2439R",
            "lat": "54.98746",
            "lon": "9.75769",
            "gnss_alt": 0,
            "baro_alt": 299,
            "track": 135,
            "speed": 87,
            "sim": "false",
            "target_dist": 65158,
            "target_dir": 214,
            "new_coordinates": {
                "10": [
                    54.98461693551021,
                    9.762644656536104,
                    299.0
                ],
                "30": [
                    54.97893020262505,
                    9.772551864312971,
                    299.0
                ],
                "60": [
                    54.97039859427967,
                    9.78740741459718,
                    299.0
                ],
                "120": [
                    54.95332995004772,
                    9.817099586449546,
                    299.0
                ],
                "180": [
                    54.936254078034075,
                    9.846766542291874,
                    299.0
                ],
                "300": [
                    54.902080693500146,
                    9.906024912839587,
                    299.0
                ]
            }
        }
    }

    print(calc_drone_aircraft_collision(drone_dict, air_traffic_dict))

    # Example usage
    # mission_test_list = [[47.397743, 8.5455945, 50.0], [47.3974938, 8.5468323, 50.0], [47.3991917, 8.5476012, 50.0], [47.3994907, 8.5462434, 50.0], [0.0, 0.0, 0.0]]

    # drone_info = {'lat': 55.4736067,
    #               'lon': 10.3271276,
    #               'alt': 97.32598437148815,
    #               'heading': 219.14,
    #               'track': 219.14,
    #               'speed': 9.60336,
    #               'speed_in_cm': 494,
    #               'new_coordinates':
    #                 {
    #                     '60': [55.471923947404576, 10.323479801038475, 97.32598437148815],
    #                 }
    #               }

    # json_string= """{
    #     "SWR445F": {
    #     "baro_alt": 36998,
    #     "flight": "SWR445F",
    #     "gnss_alt": 0,
    #     "icao_id": "4B028F",
    #     "lat": "55.89495",
    #     "lon": "9.96537",
    #     "seen": 2.083,
    #     "sim": "false",
    #     "source_name": "OUH",
    #     "source_type": "ADSB",
    #     "speed": 460,
    #     "target_dir": 334,
    #     "target_dist": 51872,
    #     "track": 184,
    #     "new_coordinates": {
    #     "60": [
    #         55.885833651259375,
    #         9.738246736566573,
    #         36998
    #         ]
    #      }
    #     },
    #     "DNU07C": {
    #     "baro_alt": 2674,
    #     "flight": "DNU07C",
    #     "gnss_alt": 0,
    #     "icao_id": "4B1F49",
    #     "lat": "55.90517",
    #     "lon": "11.13889",
    #     "seen": 0.752,
    #     "sim": "false",
    #     "source_name": "HCA",
    #     "source_type": "ADSB",
    #     "speed": 132,
    #     "target_dir": 46,
    #     "target_dist": 69818,
    #     "track": 286,
    #     "type": "AIRPLANE",
    #     "new_coordinates": {
    #     "60": [
    #         55.8699462294672,
    #         11.15689099393476,
    #         2674
    #         ]
    #         }
    #     }
    # }"""

    # # Loads the aircraft JSON string
    # json_aircraft_dict = json.loads(json_string)

    # # print(calc_mission_collision(mission_test_list, json_aircraft_dict))
    # calc_drone_aircraft_collision(drone_info, json_aircraft_dict)
