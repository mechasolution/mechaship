import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # get use_sim_time
    param_stdout = subprocess.run(
        ["ros2", "param", "get", "/robot_state_publisher", "use_sim_time"],
        capture_output=True,
        text=True,
    ).stdout
    get_use_sim_time = "true" if "True" in param_stdout else "false"
    print("use_sim_time :", get_use_sim_time)

    use_sim_time = LaunchConfiguration("use_sim_time", default=get_use_sim_time)

    # cartographer
    cartographer_config_dir = LaunchConfiguration(
        "cartographer_config_dir",
        default=os.path.join(get_package_share_directory("mechaship_slam"), "config"),
    )
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="mechaship.lua"
    )
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            configuration_basename,
        ],
    )

    # occupancy grid
    resolution = LaunchConfiguration("resolution", default="0.05")
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution",
            resolution,
            "-publish_period_sec",
            publish_period_sec,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=get_use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "cartographer_config_dir",
                default_value=cartographer_config_dir,
                description="Full path to config file to load",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value=configuration_basename,
                description="Name of lua file for cartographer",
            ),
            cartographer_node,
            DeclareLaunchArgument(
                "resolution",
                default_value=resolution,
                description="Resolution of a grid cell in the published occupancy grid",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value=publish_period_sec,
                description="OccupancyGrid publishing period",
            ),
            occupancy_grid_node,
        ]
    )
