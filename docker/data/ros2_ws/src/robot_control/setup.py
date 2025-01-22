from setuptools import setup, find_packages

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@todo.todo',
    description='Robot velocity control package for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'move_robot = robot_control.move_robot:main',
        ],
    },
)
