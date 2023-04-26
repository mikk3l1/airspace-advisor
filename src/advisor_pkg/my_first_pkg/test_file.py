my_dict = {'AFR21XY':
           {'baro_alt': 35974,
            'flight': 'AFR21XY'},
           'AFR94XK':
           {'baro_alt': 36998,
            'flight': 'AFR94XK'}
           }


print(my_dict)


for key in my_dict:
    print(my_dict[key])
    my_dict[key]['new_key'] = None

print(my_dict)