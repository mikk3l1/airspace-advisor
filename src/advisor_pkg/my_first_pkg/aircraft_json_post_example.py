#!/usr/bin/env python3

'''
Copyright 2021-2022 Kjeld Jensen
SDU UAS Center, University of Southern Denmark
kjen@mmmi.sdu.dk

2021-02-05 Kjeld Jensen First version
2021-07-08 Kjeld Jensen Updated to HealthDrone DAA at Aeroe-Svendborg
2022-01-22 Kjeld Jensen Updated post_url
'''

import requests
import json 

# parameters
post_url = 'https://healthdrone.dk/daa/hcaa/json/aircraft_json.php'
post_id_token = { 'id': 'HealthDrone', 'token': 'a234ac96' }


def get_data(post_url, post_data):

	try:
		r = requests.post(post_url, data=post_data, timeout=2.0)
		r.raise_for_status()
	except requests.exceptions.HTTPError as errh:
		print ("Error http:\n",errh)
	except requests.exceptions.ConnectionError as errc:
		print ("Error connecting:\n",errc)
	except requests.exceptions.Timeout as errt:
		print ("Error timeout:\n",errt)
	except requests.exceptions.RequestException as err:
		print ("Error something else:\n",err)
	else:
		# print(r.status_code, r.reason)
		return (r.text)
	return (False)


if __name__ == "__main__":
	json_data = get_data (post_url, post_id_token)
	#print (json_data)

	json_object = json.loads(json_data)
	print(json.dumps(json_object, indent = 1))

