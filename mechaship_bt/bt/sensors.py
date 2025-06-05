import math

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access, Status
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Bool


class UpdateScanObstacle(Behaviour):
    def __init__(self) -> None:
        super().__init__(name="UpdateScanObstacle")
        self.bb = Client(name="Mechaship")
        self.bb.register_key(key="avoid_heading_deg", access=Access.WRITE)
        self.bb.set("avoid_heading_deg", None)

        self.scan_data = None
        self.subscription = None

    def setup(self, node: Node) -> None:
        self.safe_distance = (
            node.get_parameter_or(
                "safe_distance", Parameter("safe_distance", Parameter.Type.DOUBLE, 1.5)
            )
            .get_parameter_value()
            .double_value
        )

        node.get_logger().info(f"safe_distance : {self.safe_distance}")

        self.subscription = node.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self.scan_data = (msg.ranges, msg.angle_min, msg.angle_increment)

    def update(self) -> Status:
        if not self.scan_data:
            return Status.RUNNING

        ranges, angle_min, angle_increment = self.scan_data
        avoid_angles = []

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < 0.1 or r > self.safe_distance:
                continue
            angle = math.degrees(angle_min + i * angle_increment)
            if -90.0 <= angle <= 90.0:
                avoid_angles.append(angle)

        if avoid_angles:
            avg_avoid = sum(avoid_angles) / len(avoid_angles)
            avoid_heading = (avg_avoid + 180.0) % 360.0
            self.bb.set("avoid_heading_deg", avoid_heading)
        else:
            self.bb.set("avoid_heading_deg", None)

        return Status.SUCCESS


class UpdateWaypointHeading(Behaviour):
    def __init__(self) -> None:
        super().__init__(name="UpdateWaypointHeading")
        self.bb = Client(name="Mechaship")
        self.bb.register_key(key="waypoint_reached", access=Access.WRITE)
        self.bb.register_key(key="target_heading_deg", access=Access.WRITE)

        self.bb.set("waypoint_reached", False)
        self.bb.set("target_heading_deg", 0.0)

        self.target_lat = None
        self.target_lon = None
        self.target_threshold = None

        self.lat = None
        self.lon = None
        self.subscription = None

    def setup(self, node: Node) -> None:
        self.target_lat = (
            node.get_parameter_or(
                "target_lat",
                Parameter("target_lat", Parameter.Type.DOUBLE, 37.123456),
            )
            .get_parameter_value()
            .double_value
        )
        self.target_lon = (
            node.get_parameter_or(
                "target_lon",
                Parameter("target_lon", Parameter.Type.DOUBLE, 127.123456),
            )
            .get_parameter_value()
            .double_value
        )
        self.target_threshold = (
            node.get_parameter_or(
                "target_threshold",
                Parameter("target_threshold", Parameter.Type.DOUBLE, 5.0),
            )
            .get_parameter_value()
            .double_value
        )

        node.get_logger().info(f"target_lat : {self.target_lat}")
        node.get_logger().info(f"target_lon : {self.target_lon}")
        node.get_logger().info(f"target_threshold : {self.target_threshold}")

        self.subscription = node.create_subscription(
            NavSatFix, "/gps/fix", self._gps_callback, qos_profile_sensor_data
        )

    def _gps_callback(self, msg: NavSatFix) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

        distance = self._haversine(self.lat, self.lon, self.target_lat, self.target_lon)
        self.bb.set("waypoint_reached", distance < self.target_threshold)

        heading = self._bearing(self.lat, self.lon, self.target_lat, self.target_lon)
        self.bb.set("target_heading_deg", heading)

    def update(self) -> Status:
        return Status.SUCCESS

    def _bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        # 위도, 경도 → heading(deg)
        dlon = math.radians(lon2 - lon1)
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        y = math.sin(dlon) * math.cos(lat2_r)
        x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(
            lat2_r
        ) * math.cos(dlon)
        return math.degrees(math.atan2(y, x))

    def _haversine(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        # 거리 계산
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (
            math.sin(dphi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        )
        return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))


class UpdateMarkerDetected(Behaviour):
    def __init__(self) -> None:
        super().__init__(name="UpdateMarkerDetected")
        self.bb = Client(name="Mechaship")
        self.bb.register_key(key="marker_detected", access=Access.WRITE)
        self.bb.set("marker_detected", False)

        self.subscription = None

    def setup(self, node: Node) -> None:
        self.subscription = node.create_subscription(
            Bool, "/marker_detected", self._marker_callback, qos_profile_sensor_data
        )

    def _marker_callback(self, msg: Bool) -> None:
        self.bb.set("marker_detected", msg.data)

    def update(self) -> Status:
        return Status.SUCCESS
