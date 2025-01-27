from setuptools import setup

package_name = "tricky_scenarios"

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
    description="ROS2 package for concurrency/conflict/delay testing",
    license="MIT",
    entry_points={
        "console_scripts": [
            "conflicting_publishers = tricky_scenarios.conflicting_publishers:main",
            "delayed_response = tricky_scenarios.delayed_response:main",
        ],
    },
)
