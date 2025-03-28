from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        executable="joy_node",
        package="joy",
        name="joy_node",
        namespace="",
        parameters=[],
        # debug
        output="screen",
        emulate_tty=True,
    )

    teleop_joystick = Node(
        executable="mechaship_teleop_joystick",
        package="mechaship_teleop",
        name="teleop_joystick",
        namespace="",
        parameters=[],
        # debug
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_joystick,
        ]
    )
