import os
from glob import glob

from setuptools import setup

package_name = "mechaship_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "param"),
            glob(os.path.join("param", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Max Cha",
    author_email="max@mechasolution.com",
    maintainer="Mechasolution",
    maintainer_email="techms5499@gmail.com",
    description="ROS 2 launch scripts for starting the MechaBot",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
