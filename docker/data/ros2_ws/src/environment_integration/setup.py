from setuptools import setup

package_name = 'environment_integration'

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
    description='Environment builder for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'environment_builder = environment_integration.environment_builder:main',
        ],
    },
)
