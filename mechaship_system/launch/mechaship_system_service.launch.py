import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_param_from_yaml(yaml_file_path):
    with open(yaml_file_path, "r") as file:
        params = yaml.safe_load(file)
    return params


def generate_launch_description():
    # 파라미터 폴더
    pkg_share_dir_param = os.path.join(
        get_package_share_directory("mechaship_system"), "param"
    )

    service_params_path = os.path.join(pkg_share_dir_param, "service.yaml")
    mechaship_service_parameter = LaunchConfiguration(
        "mechaship_service_parameter",
        default=os.path.join(pkg_share_dir_param, "service.yaml"),
    )
    mechaship_service_launch_arg = DeclareLaunchArgument(
        "mechaship_service_parameter",
        default_value=mechaship_service_parameter,
    )

    params = load_param_from_yaml(service_params_path)

    # YAML 파일에서 port 값 가져오기
    device_name = (
        params.get("/**", {}).get("ros__parameters", {}).get("port", "/dev/ttyMCU")
    )  # 기본값 "/dev/ttyMCU"

    # Micro-ROS Agent
    micro_ros_agent_node = Node(
        executable="micro_ros_agent",
        package="micro_ros_agent",
        name="micro_ros_agent",
        namespace="",
        arguments=["serial", "--dev", device_name],
        # debug
        output="screen",
        emulate_tty=True,
    )

    mechaship_service_node = Node(
        executable="service_node",
        package="mechaship_system",
        name="service_node",
        namespace="",
        parameters=[mechaship_service_parameter],
        # debug
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            micro_ros_agent_node,
            mechaship_service_launch_arg,
            mechaship_service_node,
        ]
    )
