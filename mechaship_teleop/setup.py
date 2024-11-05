from setuptools import find_packages, setup

package_name = "mechaship_teleop"

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
    author="Max Cha",
    author_email="max@mechasolution.com",
    maintainer="Mechasolution",
    maintainer_email="techms5499@gmail.com",
    description="Teleoperation node for MechaShip",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mechaship_teleop_keyboard = mechaship_teleop.mechaship_teleop_keyboard:main",
        ],
    },
)
