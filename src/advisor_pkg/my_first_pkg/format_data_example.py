import json
# import aircraft_json_post_example as json_post


# Consider to create a time stamp here to use for testing

def data_to_publish(json_str):
    flight_dict = {}
    traffic = json.loads(json_str)
    if traffic.get('aircraft'):
        aircraft_list = traffic['aircraft']
        for aircraft in aircraft_list:
            if aircraft.get('flight'):
                if all(key in aircraft for key in ['flight', 'lat', 'lon', 'track', 'speed']):
                    flight_name = aircraft['flight']
                    flight_dict[flight_name] = aircraft
                    continue
            
            if aircraft.get('flarm_id'):
                if all(key in aircraft for key in ['flarm_id', 'lat', 'lon', 'track', 'speed']):
                    flight_name = aircraft['flarm_id']
                    flight_dict[flight_name] = aircraft
                    continue

            if aircraft.get('icao_id'):
                if all(key in aircraft for key in ['icao_id', 'lat', 'lon', 'track', 'speed']):
                    flight_name = aircraft['icao_id']
                    flight_dict[flight_name] = aircraft
                    continue
                
    else:
        print('Value for; flight or flarm_id or icao_id key is None')
    return json.dumps(flight_dict, indent = 1)


if __name__ == "__main__":
    json_object = json.loads(json_post.get_data(json_post.post_url, json_post.post_id_token))
    json_string = json.dumps(json_object, indent = 1)
    
    print(data_to_publish(json_string))