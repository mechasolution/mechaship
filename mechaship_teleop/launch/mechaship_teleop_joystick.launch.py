from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    teleop_joystick_node = Node(
        package="mechaship_teleop",
        executable="mechaship_teleop_joystick",
        name="mechaship_teleop_joystick",
    )

    return LaunchDescription([joy_node, teleop_joystick_node])
