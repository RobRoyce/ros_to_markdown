from setuptools import find_packages, setup

package_name = "action_server"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/action", ["action/MoveToGoal.action"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YourName",
    maintainer_email="you@todo.todo",
    description="MoveToGoal action server package for ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move_to_goal = action_server.move_to_goal:main",
        ],
    },
)
