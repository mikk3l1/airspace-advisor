'''
Only for testing purposes
Simple python scripts and other stuff

Should be deleted
'''

# my_list = ['p','r','o','g','r','a','m','i','z','s']

# print(len(my_list))
# new_list = my_list[0:]
# print(new_list)


# my_list = []

# if my_list:
#     print('True')
# if not my_list:
#     print('False')


# my_list.append('index 0')
# my_list.append('index 1')
# my_list.append('index 2')


# print(my_list[1])
# print(my_list[1])
# print(my_list[2])

# new_list = ['warn', f'Collision with someti will happen in the next 60 seconds!']

# print(new_list[0])
# print(new_list[1])

# b1 = True
# b2 = False
# b3 = True
# b4 = True

# if all([b1,b2,b3,b4]):
#     print('all True')
# else:
#     print('one or more is False')



# sports = {"geeksforgeeks" : 1, "practice" : 2, "contribute" :3}
 
# # using comparison operator
# print(sports.keys() >= {"geeksforgeeks", "practice"}, '1')
# print(sports.keys() >= {"contribute", "ide"}, '2')



if __name__ == "__main__":

    dict1 = {"SAS1523": {
        "baro_alt": 38025,
        "flight": "SAS1523",
        "gnss_alt": 0,
        "icao_id": "4AC9F0",
        "lon": "9.93479",
        "seen": 0.505,
        "sim": "false",
        "source_name": "Unknown",
        "source_type": "ADSB",
        "speed": 404,
        "target_dir": 237,
        "target_dist": 29444,
        "track": 231
    },
        "SAS2987": {
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
    },
        "VKG504": {
        "baro_alt": 14948,
        "gnss_alt": 0,
        "icao_id": "45D964",
        "lat": "55.38199",
        "lon": "10.28087",
        "seen": 0.505,
        "sim": "false",
        "source_name": "HCA",
        "source_type": "ADSB",
        "speed": 408,
        "target_dir": 196,
        "target_dist": 10777,
        "track": 122
    }
    }
    # for aircraft in dict1:
    #     print(dict1[aircraft]['baro_alt'])
    #     print(dict1[aircraft].get('baro_alt'))



    #     n = len(dict1[aircraft])
    #     print(n)
    #     first_n_values = list(dict1[aircraft].values())[:n]

    #     print(first_n_values)  

        # if dict1[aircraft].keys() >= {'flight', 'lat', 'lon', 'track'}:
        #     print(dict1[aircraft], 'is GREAT!')
        # else:
        #     print(dict1[aircraft], 'dont not meet standard')
    #         print(aircraft, 'dont have flight, lat, lon, or track')
    #     else:
    #         print('we good')
        # if 'flight' not in dict1[aircraft]:
        #     print('no flight')
        # else:
        #     print(dict1[aircraft])


    # str1 = ''

    # if str1:
    #     print('here')

    # str2 = 'hello'
    # str3 = 'world'

    # str1 = f'{str2} {str3}'

    # if str1:
    #     print('here2')
    # print(str1)

    aircraft = {'1': 1, '2': 2}

    if aircraft.get('flight'):
        print('true?')
    else:
        print(bool(aircraft.get('flight')))