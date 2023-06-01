#!/usr/bin/env python3

'''
Copyright 2022 Kjeld Jensen
SDU UAS Center, University of Southern Denmark
kjen@mmmi.sdu.dk
'''

import requests
import json
from time import time

# parameters
post_url = 'https://droneweb.dk/daa/fyn/advisor/advisor_log_text.php'
post_test = { 'id': 'Mikkel', 'token': 'a244ac96', 'level' : 'info', 'text': 'This is a test'}

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
		print(r.status_code, r.reason)
		return (r.text)
	return (False)


if __name__ == "__main__":
	print (post_test)
	return_data = get_data (post_url, post_test)
	print (return_data)

	#json_object = json.loads(return_data)
	#print(json.dumps(json_object, indent = 1))

