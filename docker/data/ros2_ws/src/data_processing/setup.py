from setuptools import setup

package_name = "data_processing"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YourName",
    maintainer_email="you@todo.todo",
    description="ROS2 data processing package",
    license="MIT",
    entry_points={
        "console_scripts": [
            "data_filter = data_processing.data_filter:main",
            "data_processor = data_processing.data_processor:main",
        ],
    },
)
