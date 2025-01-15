from setuptools import setup

package_name = 'action_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # If you have a launch file, include it here.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@todo.todo',
    description='MoveToGoal action server package for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'move_to_goal = action_server.move_to_goal:main',
        ],
    },
)
