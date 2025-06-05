import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mechaship_bt"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "bt"),
            glob(os.path.join("bt/*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Sydney Kim",
    author_email="sydney@mechasolution.com",
    maintainer="Mechasolution",
    maintainer_email="techms5499@gmail.com",
    description="ROS 2 package for BT",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bt_node = mechaship_bt.bt_node:main",
        ],
    },
)
