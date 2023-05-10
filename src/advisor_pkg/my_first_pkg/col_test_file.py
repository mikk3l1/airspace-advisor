import math

EARTH_RADIUS_KM = R = 6371


def get_new_coordinates(dict: dict, time: int = 60):

    latitude = float(dict.get('lat'))
    longitude = float(dict.get('lon'))
    if dict.get('baro_alt') == None:
        altitude = dict.get('alt')
    else:
        altitude = dict.get('baro_alt')

    track = dict.get('track')
    speed = dict.get('speed')

    # Convert speed from knots to kilometers per second
    speed_kms = speed * 1.852 / 3600

    # Convert the initial direction to a bearing
    bearing = math.radians(90) - math.radians(track)

    # Calculate the distance traveled
    distance = speed_kms * time  # distance is in kilometers

    # Convert the distance to radians
    distance_rad = distance / EARTH_RADIUS_KM

    # Calculate the new latitude
    new_latitude = math.asin(math.sin(math.radians(latitude)) * math.cos(distance_rad) +
                             math.cos(math.radians(latitude)) * math.sin(distance_rad) * math.cos(bearing))

    # Calculate the new longitude
    new_longitude = math.radians(longitude) + math.atan2(math.sin(bearing) * math.sin(distance_rad) * math.cos(
        math.radians(latitude)), math.cos(distance_rad) - math.sin(math.radians(latitude)) * math.sin(new_latitude))

    # Convert the new latitude and longitude to degrees
    new_latitude_deg = math.degrees(new_latitude)
    new_longitude_deg = math.degrees(new_longitude)

    print(f"New coordinates: ({new_latitude_deg}, {new_longitude_deg})")


def correct_calc_coordinates(craft_dict: dict, time: int=60):
    lat1 = float(craft_dict.get('lat'))
    lon1 = float(craft_dict.get('lon'))
    if craft_dict.get('baro_alt') == None:
        brng_deg = craft_dict.get('alt')
    else:
        brng_deg = craft_dict.get('baro_alt')

    track = craft_dict.get('track')
    speed = craft_dict.get('speed')


    brng = math.radians(90) - math.radians(brng_deg)
    d = speed * time
    lat2 = math.asin(math.sin(math.radians(lat1))*math.cos(d/R) + math.cos(math.radians(lat1))*math.sin(d/R)*math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(math.radians(lat1)), math.cos(d/R)-math.sin(math.radians(lat1))*math.sin(lat2))

    # Convert back to degrees
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    print(f"New coordinates: ({lat2}, {lon2})")


if __name__ == "__main__":
    dict1 = {"SAS2987": {
        "baro_alt": 16998,
        "flight": "SAS2987",
        "gnss_alt": 0,
        "icao_id": "511101",
        "lat": "56.13185",
        "lon": "10.30432",
        "seen": 1.505,
        "sim": "false",
        "source_name": "OUH2",
        "source_type": "ADSB",
        "speed": 299,
        "target_dir": 359,
        "target_dist": 73015,
        "track": 151
    }
    }

    print(dict1)
    print(type(dict1))
    for aircraft in dict1:
        get_new_coordinates(dict1[aircraft])
        correct_calc_coordinates(dict1[aircraft])
