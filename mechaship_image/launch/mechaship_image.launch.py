from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # image sub 노드
    mechaship_image_sub_node = Node(
        package="mechaship_image",
        executable="mechaship_image_sub_node",
        name="mechaship_image_sub_node",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    # image color 노드
    mechaship_image_color_node = Node(
        package="mechaship_image",
        executable="mechaship_image_color_node",
        name="mechaship_image_color_node",
        output="screen",
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription([mechaship_image_sub_node, mechaship_image_color_node])
