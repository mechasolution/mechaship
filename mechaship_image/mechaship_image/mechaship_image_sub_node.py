import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class MechashipImageSub(Node):
    def __init__(self):
        super().__init__("mechaship_image_sub_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )

        self.image_subscription = self.create_subscription(
            Image, "Image", self.listener_callback, qos_profile
        )
        self.image_subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, data):
        origin_image = self.br.imgmsg_to_cv2(data, "bgr8")
        if len(origin_image):
            self.get_logger().info(str(data.header.frame_id))
            cv2.imshow("origin image", origin_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipImageSub()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
