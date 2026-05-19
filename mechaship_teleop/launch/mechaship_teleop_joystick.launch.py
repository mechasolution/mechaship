from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 조이스틱 입력
    joy_node = Node(
        executable="joy_node",
        package="joy",
        name="joy_node",
        namespace="",
        parameters=[],
        emulate_tty=True,
        # output="screen", # debug
    )

    # 조이스틱 원격 조종
    teleop_joystick = Node(
        executable="mechaship_teleop_joystick",
        package="mechaship_teleop",
        name="teleop_joystick",
        namespace="",
        parameters=[],
        emulate_tty=True,
        # output="screen", # debug
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_joystick,
        ]
    )
