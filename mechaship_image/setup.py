import glob
import os
from setuptools import setup

package_name = "mechaship_image"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sydney Kim",
    maintainer_email="sydney@mechasolution.com",
    description="ROS2 Image Node for Mechasolution Autoship 2023 Project",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mechaship_image_sub_node = mechaship_image.mechaship_image_sub_node:main",
            "mechaship_image_color_node = mechaship_image.mechaship_image_color_node:main",
        ],
    },
)
