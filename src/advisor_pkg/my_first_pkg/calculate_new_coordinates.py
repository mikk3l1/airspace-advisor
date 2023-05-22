import math
import geopy.distance

EARTH_RADIUS_KM = 6371.01

def test_me():
    print('\nPrint from collision.py\n')

# Geopy calculate the distance between two objects on a sphere e.g. Earth
def distance_between_objects(aircraft_lat_lon: list, drone_lat_lon: list) -> float:
    return  geopy.distance.great_circle(aircraft_lat_lon, drone_lat_lon).km
    

# # Calculate new coordinates for aircraft and drone after X time / default time is 60 seconds
# def get_new_coordinates(dict: dict, time: float = 60.0):

#     latitude = float(dict.get('lat'))
#     longitude = float(dict.get('lon'))
#     if dict.get('baro_alt') == None:
#         altitude = dict.get('alt')
#     else:
#         altitude = dict.get('baro_alt')

#     track = dict.get('track')
#     speed = dict.get('speed')

#     # Convert speed from knots to kilometers per second
#     speed_kms = speed * 1.852 / 3600

#     # Convert the initial direction to a bearing
#     bearing = math.radians(90) - math.radians(track)

#     # Calculate the distance traveled
#     distance = speed_kms * time # distance is in kilometers

#     # Convert the distance to radians
#     distance_rad = distance / EARTH_RADIUS_KM

#     # Calculate the new latitude
#     new_latitude = math.asin(math.sin(math.radians(latitude)) * math.cos(distance_rad) + math.cos(math.radians(latitude)) * math.sin(distance_rad) * math.cos(bearing))

#     # Calculate the new longitude
#     new_longitude = math.radians(longitude) + math.atan2(math.sin(bearing) * math.sin(distance_rad) * math.cos(math.radians(latitude)), math.cos(distance_rad) - math.sin(math.radians(latitude)) * math.sin(new_latitude))

#     # Convert the new latitude and longitude to degrees
#     new_latitude_deg = math.degrees(new_latitude)
#     new_longitude_deg = math.degrees(new_longitude)

#     return {time:[new_latitude_deg, new_longitude_deg, altitude]}


EARTH_RADIUS_NM = 3443.92  # Nautical miles
NM_PER_DEGREE_LAT = 60.0    # Nautical miles per degree of latitude
NM_PER_DEGREE_LON = 60.0 * math.cos(math.radians(0.5))  # Nautical miles per degree of longitude at equator

def calculate_new_coordinates_using_nautical_miles(aircraft: dict, time: float = 60.0):
    # Extract inputs from dictionary
    lat = float(aircraft['lat'])
    lon = float(aircraft['lon'])

    if aircraft.get('baro_alt') == None:
        altitude = aircraft['alt']
    else:
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
    new_lat_deg = math.degrees(new_lat_rad)
    new_lon_deg = math.degrees(new_lon_rad)

    # Return new coordinates as a dictionary
    return {time:[new_lat_deg, new_lon_deg, altitude]}