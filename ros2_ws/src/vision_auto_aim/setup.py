from setuptools import find_packages, setup

package_name = "vision_auto_aim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robomini-team",
    maintainer_email="robomini-team@robomini.local",
    description="ROS 2 camera viewer for the ROBOMINI simulation platform.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_view_node = vision_auto_aim.camera_view_node:main",
        ],
    },
)
