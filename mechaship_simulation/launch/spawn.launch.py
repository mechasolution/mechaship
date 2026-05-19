import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    world_name = os.environ.get("WORLD_NAME", "empty")
    model_name = os.environ.get("MODEL_NAME", "mechaship")
    x = 4.5
    y = -12.0
    z = 0.5
    yaw = 3.14

    # Gazebo에 로봇 불러오기
    spawn_entity_node = Node(
        executable="create",
        package="ros_gz_sim",
        name="create",
        namespace="",
        parameters=[
            {
                "world": world_name,
                "topic": "robot_description",
                "name": model_name,
                "x": x,
                "y": y,
                "z": z,
                "Y": yaw,
            }
        ],
        emulate_tty=True,
        # output="screen", # debug
    )

    # Gazebo reset 시 로봇 모델 respawn
    respawn_manager_node = Node(
        executable="respawn_manager_node.py",
        package="mechaship_simulation",
        name="respawn_manager",
        namespace="",
        parameters=[
            {
                "world_name": world_name,
                "model_name": model_name,
                "model_uri": f"model://{model_name}",
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
            }
        ],
        emulate_tty=True,
        # output="screen", # debug
    )

    return LaunchDescription([spawn_entity_node, respawn_manager_node])
