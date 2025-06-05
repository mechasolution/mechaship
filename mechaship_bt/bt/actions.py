import math

from geometry_msgs.msg import Vector3
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access, Status
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ComputeFinalHeading(Behaviour):
    def __init__(self, node: Node) -> None:
        super().__init__(name="ComputeFinalHeading")
        self.node = node
        self.bb = Client(name="Mechaship")
        self.bb.register_key(key="target_heading_deg", access=Access.READ)
        self.bb.register_key(key="avoid_heading_deg", access=Access.READ)

        self.publisher = self.node.create_publisher(
            Vector3, "/heading_cmd", qos_profile_sensor_data
        )

    def update(self) -> Status:
        target_heading = self.bb.get("target_heading_deg")
        avoid_heading = self.bb.get("avoid_heading_deg")

        if target_heading is None:
            self.node.get_logger().warn("Target heading is not available")
            return Status.RUNNING

        final_heading = target_heading

        if avoid_heading is not None:
            # 장애물 회피 헤딩이 있을 경우, 두 각도 사이 평균을 계산하여 우선순위 조절
            final_heading = self._weighted_average_heading(
                target_heading, avoid_heading, w1=0.8, w2=0.2
            )

        # 퍼블리시
        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = math.radians(final_heading)
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published final heading: {final_heading:.2f}°")
        return Status.SUCCESS

    def _weighted_average_heading(
        self, h1: float, h2: float, w1: float = 0.5, w2: float = 0.5
    ) -> float:
        """두 각도(도 단위)를 가중 평균하는 함수 (0~360 기준)"""
        # 라디안으로 변환
        h1_rad = math.radians(h1)
        h2_rad = math.radians(h2)

        # 벡터 합산 방식
        x = w1 * math.cos(h1_rad) + w2 * math.cos(h2_rad)
        y = w1 * math.sin(h1_rad) + w2 * math.sin(h2_rad)

        avg_rad = math.atan2(y, x)
        return (math.degrees(avg_rad) + 360.0) % 360.0
