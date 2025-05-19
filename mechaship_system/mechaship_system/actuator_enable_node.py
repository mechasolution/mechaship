import rclpy
from mechaship_interfaces.srv import ActuatorDisable, ActuatorEnable
from rclpy.node import Node
from rclpy.parameter import Parameter


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
                    "thruster_pulse_100_percentage", Parameter.Type.INTEGER, 2000
                ),
            )
            .get_parameter_value()
            .integer_value
        )

        self.__actuator_enable_client_hd = self.create_client(
            ActuatorEnable, "system/actuator/enable"
        )

        self.__actuator_disable_client_hd = self.create_client(
            ActuatorDisable, "system/actuator/disable"
        )

        msg = ActuatorEnable.Request()
        msg.key_min_degree = self.__key_min_degree
        msg.key_max_degree = self.__key_max_degree
        msg.key_pulse_0_degree = self.__key_pulse_0_degree
        msg.key_pulse_180_degree = self.__key_pulse_180_degree
        msg.thruster_pulse_0_percentage = self.__thruster_pulse_0_percentage
        msg.thruster_pulse_100_percentage = self.__thruster_pulse_100_percentage

        self.__actuator_enable_client_hd.call_async(msg)
        self.__actuator_enable_client_hd.call_async(msg)
        self.__actuator_enable_client_hd.call_async(msg)
        self.__actuator_enable_client_hd.call_async(msg)
        future = self.__actuator_enable_client_hd.call_async(msg)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("enable done")

    def shutdown(self):
        msg = ActuatorDisable.Request()

        self.__actuator_disable_client_hd.call_async(msg)
        self.__actuator_disable_client_hd.call_async(msg)
        self.__actuator_disable_client_hd.call_async(msg)
        self.__actuator_disable_client_hd.call_async(msg)
        future = self.__actuator_disable_client_hd.call_async(msg)
        rclpy.spin_until_future_complete(self, future)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorEnableNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    except Exception as e:
        node.get_logger().error(f"Exception: {e}")

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
