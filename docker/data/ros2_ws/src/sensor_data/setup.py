from setuptools import setup, find_packages

package_name = 'sensor_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sensors_only.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@todo.todo',
    description='ROS2 sensor data package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'temperature_sensor = sensor_data.temperature_sensor:main',
            'humidity_sensor = sensor_data.humidity_sensor:main',
        ],
    },
)
