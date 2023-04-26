import math
import geopy.distance
import json

EARTH_RADIUS_KM = 6371.01


def test_me():
    print('\nPrint from collision.py\n')

# Geopy calculate the distance between two objects on a sphere e.g. Earth
def distance_between_objects(aircraft_lat_lon: list, drone_lat_lon: list) -> float:
    return  geopy.distance.great_circle(aircraft_lat_lon, drone_lat_lon).km
    

# Calculate new coordinates for aircraft and drone after X time / default time is 60 seconds
def calculate_new_coordinates(aircraft_dict: dict, drone_dict: dict, time: int=60):

    # Aircraft calculations
    # Setting the aircraft variables from the aircraft dict
    aircraft_name = aircraft_dict.get('flight')
    aircraft_lat = float(aircraft_dict.get('lat'))
    aircraft_lon = float(aircraft_dict.get('lon'))
    aircraft_alt = aircraft_dict.get('baro_alt')
    aircraft_track = aircraft_dict.get('track')
    aircraft_speed = aircraft_dict.get('speed')

    # Convert aircraft speed from knots to kilometers per second
    aircraft_speed_kms = aircraft_speed * 1.852 / 3600

    # Convert the initial direction to a bearing
    aircraft_bearing = math.radians(90) - math.radians(aircraft_track)

    # Calculate the distance traveled by the plane
    aircraft_distance = aircraft_speed_kms * time # distance is in kilometers

    # Convert the distance to radians
    aircraft_distance_rad = aircraft_distance / EARTH_RADIUS_KM

    # Calculate the new latitude of the aircraft
    new_aircraft_latitude = math.asin(math.sin(math.radians(aircraft_lat)) * math.cos(aircraft_distance_rad) + math.cos(math.radians(aircraft_lat)) * math.sin(aircraft_distance_rad) * math.cos(aircraft_bearing))

    # Calculate the new longitude of the aircraft
    new_aircraft_longitude = math.radians(aircraft_lon) + math.atan2(math.sin(aircraft_bearing) * math.sin(aircraft_distance_rad) * math.cos(math.radians(aircraft_lat)), math.cos(aircraft_distance_rad) - math.sin(math.radians(aircraft_lat)) * math.sin(new_aircraft_latitude))

    # Convert the new latitude and longitude to degrees
    new_aircraft_latitude_deg = math.degrees(new_aircraft_latitude)
    new_aircraft_longitude_deg = math.degrees(new_aircraft_longitude)

    # Drone calculations
    # Setting the drone variables from the drone dict
    drone_lat = float(drone_dict.get('lat'))
    drone_lon = float(drone_dict.get('lon'))
    drone_alt = drone_dict.get('alt')
    drone_track = drone_dict.get('heading')
    drone_speed = drone_dict.get('speed')

    # Convert drone speed from knots to kilometers per second
    drone_speed_kms = drone_speed * 1.852 / 3600

    # Convert the initial drone direction to a bearing
    drone_bearing = math.radians(90) - math.radians(drone_track)

    # Calculate the distance traveled by the drone
    drone_distance_traveled = drone_speed_kms * time # distance is in kilometers

    # Convert the distance to radians
    drone_distance_rad = drone_distance_traveled / EARTH_RADIUS_KM

    # Calculate the new latitude of the aircraft
    new_drone_latitude = math.asin(math.sin(math.radians(drone_lat)) * math.cos(drone_distance_rad) + math.cos(math.radians(drone_lat)) * math.sin(drone_distance_rad) * math.cos(drone_bearing))

    # Calculate the new longitude of the aircraft
    new_drone_longitude = math.radians(drone_lon) + math.atan2(math.sin(drone_bearing) * math.sin(drone_distance_rad) * math.cos(math.radians(drone_lat)), math.cos(drone_distance_rad) - math.sin(math.radians(drone_lat)) * math.sin(new_drone_latitude))

    # Convert the new latitude and longitude to degrees
    new_drone_latitude_deg = math.degrees(new_drone_latitude)
    new_drone_longitude_deg = math.degrees(new_drone_longitude)

    # Calculate the altitude difference between aircraft and drone
    altitude_between_aircraft_drone = abs((aircraft_alt - drone_alt))

    # Calculale distance in KM between the aircraft and drone using the "geopy" python3 library
    distance_between_aircraft_drone = distance_between_objects((new_aircraft_latitude_deg,new_aircraft_longitude_deg),(new_drone_latitude_deg,new_drone_longitude_deg))

    # TODO In the future, maybe return something else
    if  altitude_between_aircraft_drone < 100 and distance_between_aircraft_drone < 10:
        return f'WARNING! Carefull of {aircraft_name}, it might be on your path in {time} seconds!\n'
    else:
        return f'Aircraft {aircraft_name} should not be in your path the next {time} seconds, if you both fly straight.\n'
    # if  altitude_between_aircraft_drone < 100 and distance_between_aircraft_drone < 10:
    #     print(f'WARNING! Carefull of {aircraft_name}, it might be on your path in {time} seconds!')
    # else:
    #     print(f'Aircraft {aircraft_name} should not be in your path the next {time} seconds, if you both fly straight.')


def get_advise(aircraft_dict, drone_dict):
    advise = ''
    for aircraft in aircraft_dict:
        if aircraft_dict.get(aircraft).get('lat') == None and aircraft_dict.get(aircraft).get('lon') == None:
            advise += f'{aircraft_dict.get(aircraft).get("flight")} is missing latitude and longitude\n'
        else:
            advise += calculate_new_coordinates(aircraft_dict.get(aircraft), drone_dict)
    return advise

# Used for testing
def json_to_dict(json_string: str):
    test_drone_dict = {
    "lat" : 37.8199,
    "lon" : -122.4783,
    "alt" : 12000,
    "heading" : 135,
    "speed" : 55 ,
    }   
    json_aircraft_dict = json.loads(json_string)
    for name in json_aircraft_dict:
        calculate_new_coordinates(json_aircraft_dict.get(name), test_drone_dict)

if __name__ == '__main__':
    # Example usage
    drone_dict = {
        "lat" : 55.05041,
        "lon" : 10.573178,
        "alt" : 12000,
        "heading" : 135,
        "speed" : 55 ,
    }


    json_string= """{
        "FIN7KL": {
            "baro_alt": 40997,
            "flight": "FIN7KL",
            "gnss_alt": 0,
            "icao_id": "461F55",
            "lat": "55.80501",
            "lon": "11.04444",
            "seen": 31.172,
            "sim": "false",
            "source_name": "Unknown",
            "source_type": "ADSB",
            "speed": 511,
            "target_dir": 51,
            "target_dist": 58066,
            "track": 53
        },
        "NSZ5421": {
            "baro_alt": 35997,
            "flight": "NSZ5421",
            "gnss_alt": 0,
            "icao_id": "4ACA14",
            "lat": "54.91969",
            "lon": "9.52006",
            "seen": 115.172,
            "sim": "false",
            "source_name": "Unknown",
            "source_type": "ADSB",
            "speed": 445,
            "target_dir": 220,
            "target_dist": 80244,
            "track": 224
        }
    }"""

    # Loads the aircraft JSON string
    json_aircraft_dict = json.loads(json_string)

    for name in json_aircraft_dict:
        calculate_new_coordinates(json_aircraft_dict.get(name), drone_dict)



