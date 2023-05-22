import json
# import aircraft_json_post_example as json_post


# Consider to create a time stamp here to use for testing

def data_to_publish(json_str):
    flight_dict = {}
    traffic = json.loads(json_str)
    if traffic.get('aircraft'):
        aircraft_list = traffic['aircraft']
        for aircraft in aircraft_list:
            if aircraft['flight'] == None:
                continue                
            if all(key in aircraft for key in ['flight', 'lat', 'lon', 'track', 'speed']):
                flight_name = aircraft['flight']
                flight_dict[flight_name] = aircraft
    else:
        print('Value for "flight" key is None')
    return json.dumps(flight_dict)


if __name__ == "__main__":
    json_object = json.loads(json_post.get_data(json_post.post_url, json_post.post_id_token))
    json_string = json.dumps(json_object, indent = 1)
    

    print(data_to_publish(json_string))
