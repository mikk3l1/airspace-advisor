from setuptools import setup

package_name = 'my_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mikkel',
    maintainer_email='mikkel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_first_pkg.publisher:main',
            'listener = my_first_pkg.subscriber:main',
            'advisor = my_first_pkg.mavros_drone_info:main',
            'test_sub = my_first_pkg.sub:main',
        ],
    },
)
