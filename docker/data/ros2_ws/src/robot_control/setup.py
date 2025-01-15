from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
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
