from setuptools import setup

import os
from glob import glob

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mikkel',
    maintainer_email='mihoe18@student.sdu.dk',
    description='Airspace advisor for Airtrack',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_first_pkg.publisher:main',
            'listener = my_first_pkg.subscriber:main',
            'collision_advisor = my_first_pkg.collision_advisor:main',
            'qgc_pub = my_first_pkg.air_traffic_to_qgc:main',
            'tts = my_first_pkg.tts_node:main'
        ],
    },
)
