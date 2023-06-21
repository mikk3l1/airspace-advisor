aircraft = {
    "now": 1686922044.639,
    "aircraft": [
        {
            "seen": 14.639,
            "source_type": "OGN",
            "source_name": "SafeSky",
            "icao_id": "459299",
            "lat": "55.75602",
            "lon": "9.76055",
            "gnss_alt": 3188,
            "track": 129,
            "speed": 96,
            "sim": "false",
            "target_dist": 47317,
            "target_dir": 312
        },
        {
            "seen": 0.639,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "45AB21",
            "flight": "SAS596",
            "lat": "55.87754",
            "lon": "10.70317",
            "gnss_alt": 0,
            "baro_alt": 23924,
            "track": 101,
            "speed": 340,
            "sim": "false",
            "target_dist": 50565,
            "target_dir": 28
        },
        {
            "seen": 0.639,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "3CD71C",
            "flight": "PWF7711",
            "lat": "55.69963",
            "lon": "9.61179",
            "gnss_alt": 0,
            "baro_alt": 26998,
            "track": 166,
            "speed": 338,
            "sim": "false",
            "target_dist": 51395,
            "target_dir": 299
        },
        {
            "seen": 1.639,
            "source_type": "ADSB",
            "source_name": "OUH2",
            "icao_id": "4CC27A",
            "flight": "ICE98C",
            "lat": "55.84304",
            "lon": "11.05431",
            "gnss_alt": 0,
            "baro_alt": 22149,
            "track": 94,
            "speed": 349,
            "sim": "false",
            "target_dist": 61267,
            "target_dir": 48
        },
        {
            "seen": 0.639,
            "source_type": "ADSB",
            "source_name": "Unknown",
            "icao_id": "40688A",
            "flight": "BAW815C",
            "lat": "54.95399",
            "lon": "10.92749",
            "gnss_alt": 0,
            "baro_alt": 25774,
            "track": 237,
            "speed": 456,
            "sim": "false",
            "target_dist": 69370,
            "target_dir": 146
        },
        {
            "seen": 1.639,
            "source_type": "ADSB",
            "source_name": "OUH2",
            "icao_id": "503CB8",
            "flight": "KLJ8282",
            "lat": "55.89699",
            "lon": "11.19644",
            "gnss_alt": 0,
            "baro_alt": 36998,
            "track": 144,
            "speed": 435,
            "sim": "false",
            "target_dist": 71897,
            "target_dir": 49
        },
        {
            "seen": 0.639,
            "source_type": "ADSB",
            "source_name": "OUH",
            "icao_id": "4CA766",
            "flight": "RYR8YY",
            "lat": "55.92796",
            "lon": "9.42785",
            "gnss_alt": 0,
            "baro_alt": 35525,
            "track": 91,
            "speed": 414,
            "sim": "false",
            "target_dist": 75540,
            "target_dir": 312
        }
    ],
    "target": "wpt"
}

for craft in aircraft["aircraft"]:
    # TODO
    pass

# aircraft['baro_alt'] = aircraft['baro_alt'] * 0.3048
# print(aircraft['baro_alt'])

# if float(aircraft.get('baro_alt')) < 3050:
#     print('danger!')
