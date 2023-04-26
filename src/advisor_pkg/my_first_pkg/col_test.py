import math
import geopy.distance

EARTH_RADIUS_KM = 6371.01

def test_me():
    print('\nPrint from collision.py\n')

# Geopy calculate the distance between two objects on a sphere e.g. Earth
def distance_between_objects(aircraft_lat_lon: list, drone_lat_lon: list) -> float:
    return  geopy.distance.great_circle(aircraft_lat_lon, drone_lat_lon).km
    

# Calculate new coordinates for aircraft and drone after X time / default time is 60 seconds
def get_new_coordinates(dict: dict, time: int=60):

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

    return {time:[new_latitude_deg, new_longitude_deg, altitude]}