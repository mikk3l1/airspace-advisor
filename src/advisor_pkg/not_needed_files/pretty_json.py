import json

json_data = """{'baro_alt': 40997, 'flight': 'FIN7KL', 'gnss_alt': 0, 'icao_id': '461F55', 'lat': '55.80501', 'lon': '11.04444', 'seen': 31.172, 'sim': 'false', 'source_name': 'Unknown', 'source_type': 'ADSB', 'speed': 511, 'target_dir': 51, 'target_dist': 58066, 'track': 53, 'new_coordinates': {60.0: [55.89011630292254, 11.246227549539025, 40997.0]}}"""

json_object = json.loads(json_data)

json_formatted_str = json.dumps(json_object, indent=2)

print(json_formatted_str)