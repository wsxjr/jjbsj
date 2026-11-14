from setuptools import find_packages, setup

package_name = "fire_test"

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
    maintainer="wsxjr",
    maintainer_email="3383722558@qq.com",
    description="Continuous fire test node for the ROBOMINI simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "continuous_fire_node = fire_test.continuous_fire_node:main",
        ],
    },
)
