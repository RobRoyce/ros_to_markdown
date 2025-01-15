from setuptools import setup

package_name = 'data_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # If we had a launch file here, we'd list it in data_files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@todo.todo',
    description='ROS2 data_processing package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'data_filter = data_processing.data_filter:main',
            'data_processor = data_processing.data_processor:main',
        ],
    },
)
