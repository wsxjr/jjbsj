from glob import glob

from setuptools import find_packages, setup

package_name = "target_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wsxjr",
    maintainer_email="3383722558@qq.com",
    description="ROS 2 target tracking package for the ROBOMINI simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tracker_node = target_tracker.tracker_node:main",
        ],
    },
)
