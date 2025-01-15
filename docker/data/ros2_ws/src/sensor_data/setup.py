from setuptools import setup

package_name = 'sensor_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@todo.todo',
    description='ROS2 sensor_data package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'temperature_publisher = sensor_data.temperature_publisher:main',
            'humidity_publisher = sensor_data.humidity_publisher:main',
        ],
    },
)
