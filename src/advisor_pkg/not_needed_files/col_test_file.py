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
    distance = speed_kms * time # distance is in kilometers

    # Convert the distance to radians
    distance_rad = distance / EARTH_RADIUS_KM

    # Calculate the new latitude
    new_latitude = math.asin(math.sin(math.radians(latitude)) * math.cos(distance_rad) + math.cos(math.radians(latitude)) * math.sin(distance_rad) * math.cos(bearing))

    # Calculate the new longitude
    new_longitude = math.radians(longitude) + math.atan2(math.sin(bearing) * math.sin(distance_rad) * math.cos(math.radians(latitude)), math.cos(distance_rad) - math.sin(math.radians(latitude)) * math.sin(new_latitude))

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


# EARTH_RADIUS_NM = 3440.07  # Nautical miles
EARTH_RADIUS_NM = 3443.92  # Nautical miles
NM_PER_DEGREE_LAT = 60.0    # Nautical miles per degree of latitude
NM_PER_DEGREE_LON = 60.0 * math.cos(math.radians(0.5))  # Nautical miles per degree of longitude at equator

def calculate_new_coordinates(aircraft: dict, time: float=60.0):
    # Extract inputs from dictionary
    lat = float(aircraft['lat'])
    lon = float(aircraft['lon'])
    print(aircraft.get('baro_alt'))
    altitude = aircraft['baro_alt']
    track = float(aircraft['track'])
    speed = float(aircraft['speed'])

    # Calculate distance traveled
    distance_nm = (speed / 3600.0) * time   # Convert speed from knots to nautical miles per minute
    distance_deg_lat = distance_nm / NM_PER_DEGREE_LAT  # Convert distance in nautical miles to degrees of latitude
    distance_deg_lon = distance_nm / NM_PER_DEGREE_LON  # Convert distance in nautical miles to degrees of longitude at current latitude

    # Calculate new latitude and longitude
    print((math.cos(math.radians(track)) * distance_deg_lat))
    new_lat = lat + (math.cos(math.radians(track)) * distance_deg_lat)   # Calculate new latitude based on current latitude, track, and distance traveled
    new_lon = lon + (math.sin(math.radians(track)) * distance_deg_lon)   # Calculate new longitude based on current longitude, track, and distance traveled

    # Calculate new altitude
    new_altitude = altitude

    # Calculate new track
    new_track = track

    # Calculate new speed (assuming constant speed)
    new_speed = speed

    # Return new coordinates as a dictionary
    return {
        'lat': new_lat,
        'lon': new_lon,
        'altitude': new_altitude,
        'track': new_track,
        'speed': new_speed
    }


def calculate_new_coordinates_using_nautical_miles(aircraft: dict, time: float=60.0):
    # Extract inputs from dictionary
    lat = float(aircraft['lat'])
    lon = float(aircraft['lon'])
    altitude = aircraft['baro_alt']
    track = float(aircraft['track'])
    speed = float(aircraft['speed'])

    # Calculate distance traveled
    distance_nm = (speed / 3600.0) * time   # Convert speed from knots to nautical miles per minute
    distance_rad = distance_nm / EARTH_RADIUS_NM   # Convert distance in nautical miles to radians of arc length
    lat_rad = math.radians(lat)  # Convert latitude to radians
    lon_rad = math.radians(lon)  # Convert longitude to radians
    track_rad = math.radians(track)  # Convert track to radians
    cos_lat = math.cos(lat_rad)   # Precompute cosine of latitude for efficiency
    sin_lat = math.sin(lat_rad)   # Precompute sine of latitude for efficiency

    # Calculate new latitude and longitude
    new_lat_rad = math.asin(sin_lat * math.cos(distance_rad) + cos_lat * math.sin(distance_rad) * math.cos(track_rad))  # Calculate new latitude in radians based on current latitude, track, and distance traveled
    new_lon_rad = lon_rad + math.atan2(math.sin(track_rad) * math.sin(distance_rad) * cos_lat, math.cos(distance_rad) - sin_lat * math.sin(new_lat_rad))  # Calculate new longitude in radians based on current longitude, track, and distance traveled

    # Convert new latitude and longitude back to decimal degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    # Calculate new altitude
    new_altitude = altitude

    # Calculate new track
    new_track = track

    # Calculate new speed (assuming constant speed)
    new_speed = speed

    # Return new coordinates as a dictionary
    return {
        'lat': new_lat,
        'lon': new_lon,
        'altitude': new_altitude,
        'track': new_track,
        'speed': new_speed
    }


# "lat": "55.4703492",
#  "lon": "10.3170954",


if __name__ == "__main__":
    dict1 = {"SAS2987": {
        "baro_alt": 16998,
        "flight": "SAS2987",
        "gnss_alt": 0,
        "icao_id": "511101",
        "lat": "55.4703492",
        "lon": "10.3170954",
        "seen": 1.505,
        "sim": "false",
        "source_name": "OUH2",
        "source_type": "ADSB",
        "speed": 9.719999999999999,
        "target_dir": 359,
        "target_dist": 73015,
        "track": 239.48
    }
    }

    # print(dict1)
    # print(type(dict1))
    for aircraft in dict1:
        get_new_coordinates(dict1[aircraft])
        correct_calc_coordinates(dict1[aircraft])
        print(calculate_new_coordinates(dict1[aircraft]))
        print(calculate_new_coordinates_with_earth(dict1[aircraft]), 'this works')
