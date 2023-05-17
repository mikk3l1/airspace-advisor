import geopy.distance
import json


# Geopy calculate the distance in kilometers between two objects on a sphere e.g. Earth
def distance_between_objects(object1: list, object2: list) -> float:
    return  geopy.distance.great_circle(object1, object2).km

def calc_mission_collision(mission_waypoints: list, air_traffic_dict: dict) -> list:
    waypoint_aircraft_distance_dict = {}
    waypoint_aircraft_collision = ''

    for aircraft in air_traffic_dict:
        aircraft_new_coordinates_list = air_traffic_dict[aircraft]['new_coordinates'].get('60.0')

    
        for index, waypoint in enumerate(mission_waypoints[0:len(mission_waypoints)-1]):
            
            waypoint_aircraft_distance_dict[aircraft] = distance_between_objects(waypoint[0:2], aircraft_new_coordinates_list[0:2])

        # FFA well-clear
        # 2_000 ft horizontal (0.6096 km)
        # 600 ft vertical (182.88 meter)
        if any(waypoint_aircraft_distance_dict) < 1 and waypoint[2]-aircraft_new_coordinates_list[2] < 200:
            waypoint_aircraft_collision += f'{aircraft},'

    if waypoint_aircraft_collision:
        return ['warn', f'Carefull! {waypoint_aircraft_collision} will collide with your mission within the next 60 seconds!']
    else:
        return ['info', f'No Aircraft(s) will collide with the mission waypoints']

def calc_drone_aircraft_collision(drone_dict: dict, air_traffic_dict: dict) -> list:
    drone_aircraft_collision = []
    drone_new_coordinates_lat_long = drone_dict['new_coordinates'].get('60.0')

    for aircraft in air_traffic_dict:
        aircraft_new_coordinates_list_lat_long = air_traffic_dict[aircraft]['new_coordinates'].get('60.0')

        distance_between_drone_aircraft = distance_between_objects(drone_new_coordinates_lat_long[0:2], aircraft_new_coordinates_list_lat_long[0:2])
        altitude_between_drone_aircraft = abs(drone_new_coordinates_lat_long[2:3][0])-abs(aircraft_new_coordinates_list_lat_long[2:3][0])

        # FFA well-clear
        # 2_000 ft horizontal (0.6096 kilometer / 609,6 meter)
        # 600 ft vertical (0,18288 kilometer / 182.88 meter)
        if distance_between_drone_aircraft < 1 and altitude_between_drone_aircraft < 200:
            drone_aircraft_collision.append(aircraft)

    if drone_aircraft_collision:
        return ['warn', f'Collision with {drone_aircraft_collision} will happen in the next 60 seconds!']
    else:
        return ['info', f'No Aircraft will collide with the drone in the next 60 seconds']


if __name__ == '__main__':
    # Example usage
    mission_test_list = [[47.397743, 8.5455945, 50.0], [47.3974938, 8.5468323, 50.0], [47.3991917, 8.5476012, 50.0], [47.3994907, 8.5462434, 50.0], [0.0, 0.0, 0.0]]

    drone_info = {'lat': 55.4736067, 
                  'lon': 10.3271276, 
                  'alt': 97.32598437148815, 
                  'heading': 219.14, 
                  'track': 219.14, 
                  'speed': 9.60336, 
                  'speed_in_cm': 494, 
                  'new_coordinates':
                    {
                        '60': [55.471923947404576, 10.323479801038475, 97.32598437148815],
                    }
                  }

    json_string= """{
        "SWR445F": {
        "baro_alt": 36998,
        "flight": "SWR445F",
        "gnss_alt": 0,
        "icao_id": "4B028F",
        "lat": "55.89495",
        "lon": "9.96537",
        "seen": 2.083,
        "sim": "false",
        "source_name": "OUH",
        "source_type": "ADSB",
        "speed": 460,
        "target_dir": 334,
        "target_dist": 51872,
        "track": 184,
        "new_coordinates": {
        "60": [
            55.885833651259375,
            9.738246736566573,
            36998
            ]
         }
        },
        "DNU07C": {
        "baro_alt": 2674,
        "flight": "DNU07C",
        "gnss_alt": 0,
        "icao_id": "4B1F49",
        "lat": "55.90517",
        "lon": "11.13889",
        "seen": 0.752,
        "sim": "false",
        "source_name": "HCA",
        "source_type": "ADSB",
        "speed": 132,
        "target_dir": 46,
        "target_dist": 69818,
        "track": 286,
        "type": "AIRPLANE",
        "new_coordinates": {
        "60": [
            55.8699462294672,
            11.15689099393476,
            2674
            ]
            }
        }
    }"""

    # Loads the aircraft JSON string
    json_aircraft_dict = json.loads(json_string)

    # print(calc_mission_collision(mission_test_list, json_aircraft_dict))
    calc_drone_aircraft_collision(drone_info, json_aircraft_dict)
