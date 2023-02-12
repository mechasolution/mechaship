import os
from setuptools import setup
from glob import glob

package_name = "mechaship_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="parallels",
    maintainer_email="dhksrl0508@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mechaship_teleop_keyboard = mechaship_teleop.mechaship_teleop_keyboard:main",
            "mechaship_teleop_joystick = mechaship_teleop.mechaship_teleop_joystick:main",
        ],
    },
)
