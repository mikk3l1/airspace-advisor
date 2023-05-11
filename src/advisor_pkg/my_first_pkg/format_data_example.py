import json
# import state_test
# import aircraft_json_post_example as json_post

# current_state = state_test.AirTrafficAdvisory()


def data_to_publish(json_str):
    flight_dict = {}
    # print('checking if key exists in traffic_json')
    traffic = json.loads(json_str)
    if not (traffic.get('aircraft') is None):
        # Get the 'aircraft' list
        aircraft_list = traffic.get('aircraft')
        # Iterate over each aircraft from the JSON object
        for aircraft in aircraft_list:
            # Remove aircraft that dont have 'flight', 'lat', 'lon', 'track'
            if not aircraft.keys() >= {'flight', 'lat', 'lon', 'track', 'speed'}:
                continue
            # Find each aircrafts name using the 'flight' key
            flight_name = aircraft['flight']
            # Add the aircraft to a dict to keep the track of each of them
            flight_dict.update({flight_name: aircraft})
    else:
        print('value for "flight" key is None')
    return json.dumps(flight_dict, sort_keys=True, indent=1)



if __name__ == "__main__":
    json_object = json.loads(json_post.get_data(json_post.post_url, json_post.post_id_token))
    json_string = json.dumps(json_object, indent = 1)
    

    print(data_to_publish(json_string))
