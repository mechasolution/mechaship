from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    teleop_joystick = Node(
        package="mechaship_teleop",
        executable="mechaship_teleop_joystick",
        name="teleop_joystick",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_joystick,
        ]
    )
