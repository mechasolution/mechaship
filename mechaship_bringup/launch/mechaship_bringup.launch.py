import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # 파라미터 폴더
    pkg_share_dir_param = os.path.join(
        get_package_share_directory("mechaship_bringup"), "param"
    )

    # USB 카메라 드라이버
    mechaship_camera_parameter = LaunchConfiguration(
        "mechaship_camera_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_camera.yaml"),
    )
    usb_camera_launch_arg = DeclareLaunchArgument(
        "mechaship_camera_parameter", default_value=mechaship_camera_parameter
    )
    usb_camera_node = Node(
        executable="usb_cam_node_exe",
        package="usb_cam",
        name="usb_cam_node_exe",
        namespace="/",
        parameters=[mechaship_camera_parameter],
        # debug
        output="screen",
        emulate_tty=True,
    )

    # LiDAR 드라이버
    mechaship_lidar_parameter = LaunchConfiguration(
        "mechaship_lidar_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_lidar.yaml"),
    )
    lidar_driver_launch_arg = DeclareLaunchArgument(
        "mechaship_lidar_parameter", default_value=mechaship_lidar_parameter
    )
    lidar_driver_node = LifecycleNode(
        executable="ydlidar_ros2_driver_node",
        package="ydlidar_ros2_driver",
        name="ydlidar_ros2_driver_node",
        namespace="",
        parameters=[mechaship_lidar_parameter],
        # debug
        output="screen",
        emulate_tty=True,
    )

    # IMU 드라이버
    mechaship_imu_parameter = LaunchConfiguration(
        "mechaship_imu_parameter",
        default=os.path.join(pkg_share_dir_param, "mechaship_imu.yaml"),
    )
    imu_driver_launch_arg = DeclareLaunchArgument(
        "mechaship_imu_parameter", default_value=mechaship_imu_parameter
    )
    imu_driver_node = Node(
        executable="iahrs_ros2_driver_node",
        package="iahrs_ros2_driver",
        name="iahrs_ros2_driver",
        namespace="",
        parameters=[mechaship_imu_parameter],
        remappings=[("imu/data", "imu")],
        # debug
        output="screen",
        emulate_tty=True,
    )

    # 로봇 형상 정보 publish
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mechaship_description"),
                "launch",
                "mechaship_state_publisher.launch.py",
            )
        )
    )

    # LiDAR 기반 Odometry publish
    rf2o_laser_odometry_node = Node(
        executable="rf2o_laser_odometry_node",
        package="rf2o_laser_odometry",
        name="rf2o_laser_odometry",
        namespace="/",
        parameters=[
            {
                "laser_scan_topic": "scan",
                "odom_topic": "odom",
                "publish_tf": True,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 10.0,
            }
        ],
        # debug
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            usb_camera_launch_arg,
            usb_camera_node,
            lidar_driver_launch_arg,
            lidar_driver_node,
            imu_driver_launch_arg,
            imu_driver_node,
            robot_state_publisher,
            rf2o_laser_odometry_node,
        ]
    )
