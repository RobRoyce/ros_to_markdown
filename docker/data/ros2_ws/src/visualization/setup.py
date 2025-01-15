from setuptools import setup

package_name = 'visualization'

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
    description='ROS2 visualization/logging package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'visualize_data = visualization.visualize_data:main',
        ],
    },
)
