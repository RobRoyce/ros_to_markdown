from setuptools import setup

package_name = "user_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YourName",
    maintainer_email="you@todo.todo",
    description="Teleoperation or user command interface for ROS2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "command_listener = user_interface.command_listener:main",
        ],
    },
)
