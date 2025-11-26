import rclpy
from bt.actions import *
from bt.conditions import *
from bt.sensors import *
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence
from py_trees.display import unicode_blackboard, unicode_tree
from py_trees_ros.trees import BehaviourTree
from rclpy.node import Node


class BTNode(Node):
    def __init__(self) -> None:
        super().__init__("bt_node")

        # 트리 구성
        self.tree = BehaviourTree(root=self._create_behavior_tree())
        self.tree.setup(node=self, timeout=10.0)
        self.create_timer(1.0, self._tick_tree)

    def _tick_tree(self) -> None:
        self.tree.tick()
        # debug
        self.get_logger().info(unicode_blackboard())
        self.get_logger().info(unicode_tree(self.tree.root, show_status=True))

    def _create_behavior_tree(self) -> Parallel:
        # 센서 업데이트
        sensor_parallel = Parallel(
            name="SensorUpdates", policy=ParallelPolicy.SuccessOnAll()
        )
        sensor_parallel.add_children(
            [UpdateScanObstacle(), UpdateWaypointHeading(), UpdateMarkerDetected()]
        )

        # 미션 시퀀스
        mission_sequence = Sequence(name="MissionSequence", memory=True)
        mission_sequence.add_children([IsWaypointReached(), IsMarkerDetected()])

        # 헤딩 계산
        final_heading_action = ComputeFinalHeading(self)

        # 루트
        root = Parallel(name="RootParallel", policy=ParallelPolicy.SuccessOnAll())
        root.add_children([sensor_parallel, mission_sequence, final_heading_action])

        return root


def main(args=None):
    rclpy.init(args=args)
    node = BTNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    except Exception as e:
        node.get_logger().error(f"Exception: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
