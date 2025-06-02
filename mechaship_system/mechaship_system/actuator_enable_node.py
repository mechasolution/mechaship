import time

import rclpy
from mechaship_interfaces.srv import ActuatorEnable
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros2node.api import get_node_names


class ActuatorEnableNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "actuator_enable_node", automatically_declare_parameters_from_overrides=True
        )

        # 파라미터 가져오기
        self.__key_min_degree = (
            self.get_parameter_or(
                "key_min_degree",
                Parameter("key_min_degree", Parameter.Type.DOUBLE, 0.0),
            )
            .get_parameter_value()
            .double_value
        )

        self.__key_max_degree = (
            self.get_parameter_or(
                "key_max_degree",
                Parameter("key_max_degree", Parameter.Type.DOUBLE, 180.0),
            )
            .get_parameter_value()
            .double_value
        )

        self.__key_pulse_0_degree = (
            self.get_parameter_or(
                "key_pulse_0_degree",
                Parameter("key_pulse_0_degree", Parameter.Type.INTEGER, 500),
            )
            .get_parameter_value()
            .integer_value
        )

        self.__key_pulse_180_degree = (
            self.get_parameter_or(
                "key_pulse_180_degree",
                Parameter("key_pulse_180_degree", Parameter.Type.INTEGER, 2500),
            )
            .get_parameter_value()
            .integer_value
        )

        self.__thruster_pulse_0_percentage = (
            self.get_parameter_or(
                "thruster_pulse_0_percentage",
                Parameter("thruster_pulse_0_percentage", Parameter.Type.INTEGER, 1500),
            )
            .get_parameter_value()
            .integer_value
        )

        self.__thruster_pulse_100_percentage = (
            self.get_parameter_or(
                "thruster_pulse_100_percentage",
                Parameter(
                    "thruster_pulse_100_percentage", Parameter.Type.INTEGER, 1900
                ),
            )
            .get_parameter_value()
            .integer_value
        )

        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.create_timer(
            0.4, self.__watch_node_timer_callback, callback_group=self.timer_cb
        )
        self.__last_node_status = False
        self.__last_warning_time = 0.0

        self.msg = ActuatorEnable.Request()

    def __enable_actuator(self):
        self.__actuator_enable_client_hd = self.create_client(
            ActuatorEnable, "system/actuator/enable"
        )
        while not self.__actuator_enable_client_hd.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                "actuator service not available, waiting again..."
            )

        self.msg.key_min_degree = self.__key_min_degree
        self.msg.key_max_degree = self.__key_max_degree
        self.msg.key_pulse_0_degree = self.__key_pulse_0_degree
        self.msg.key_pulse_180_degree = self.__key_pulse_180_degree
        self.msg.thruster_pulse_0_percentage = self.__thruster_pulse_0_percentage
        self.msg.thruster_pulse_100_percentage = self.__thruster_pulse_100_percentage

        self.future = self.__actuator_enable_client_hd.call_async(self.msg)
        self.executor.spin_until_future_complete(self.future)

        self.get_logger().info("actuator enabled")

        self.__actuator_enable_client_hd.destroy()

    def __watch_node_timer_callback(self):
        available_nodes = get_node_names(node=self)
        for name, namespace, full_name in available_nodes:
            if name == "mcu_node":
                if self.__last_node_status is False:
                    self.__last_node_status = True
                    self.__last_warning_time = 0
                    self.__enable_actuator()
                return
        self.__last_node_status = False

        current_time = time.time()
        if current_time - self.__last_warning_time >= 5.0:
            self.get_logger().warning(
                "MCU 노드를 찾을 수 없습니다. 아래 내용을 점검해주세요."
            )
            self.get_logger().warning("\t 1. 조종기 설정이 UROS 모드인지 확인해주세요.")
            self.get_logger().warning(
                "\t 2. SBC와 메인보드 간의 USB 케이블이 올바르게 연결되어 있는지 확인해주세요."
            )
            self.__last_warning_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorEnableNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
