import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
import message_filters

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

names = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    4: "airplane",
    5: "bus",
    6: "train",
    7: "truck",
    8: "boat",
    9: "traffic light",
    10: "fire hydrant",
    11: "stop sign",
    12: "parking meter",
    13: "bench",
    14: "bird",
    15: "cat",
    16: "dog",
    17: "horse",
    18: "sheep",
    19: "cow",
    20: "elephant",
    21: "bear",
    22: "zebra",
    23: "giraffe",
    24: "backpack",
    25: "umbrella",
    26: "handbag",
    27: "tie",
    28: "suitcase",
    29: "frisbee",
    30: "skis",
    31: "snowboard",
    32: "sports ball",
    33: "kite",
    34: "baseball bat",
    35: "baseball glove",
    36: "skateboard",
    37: "surfboard",
    38: "tennis racket",
    39: "bottle",
    40: "wine glass",
    41: "cup",
    42: "fork",
    43: "knife",
    44: "spoon",
    45: "bowl",
    46: "banana",
    47: "apple",
    48: "sandwich",
    49: "orange",
    50: "broccoli",
    51: "carrot",
    52: "hot dog",
    53: "pizza",
    54: "donut",
    55: "cake",
    56: "chair",
    57: "couch",
    58: "potted plant",
    59: "bed",
    60: "dining table",
    61: "toilet",
    62: "tv",
    63: "laptop",
    64: "mouse",
    65: "remote",
    66: "keyboard",
    67: "cell phone",
    68: "microwave",
    69: "oven",
    70: "toaster",
    71: "sink",
    72: "refrigerator",
    73: "book",
    74: "clock",
    75: "vase",
    76: "scissors",
    77: "teddy bear",
    78: "hair drier",
    79: "toothbrush",
}


class VisualizeNode(Node):
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    COLOR = (0, 255, 0)
    THICKNESS = 2

    def __init__(self) -> None:
        super().__init__(
            "visualize_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.image_topic = (
            self.get_parameter_or(
                "image_topic",
                Parameter("image_topic", Parameter.Type.STRING, "/image_raw"),
            )
            .get_parameter_value()
            .string_value
        )

        self.detections_topic = (
            self.get_parameter_or(
                "detections_topic",
                Parameter("detections_topic", Parameter.Type.STRING, "/detections"),
            )
            .get_parameter_value()
            .string_value
        )

        self.preview = (
            self.get_parameter_or(
                "preview",
                Parameter("preview", Parameter.Type.BOOL, True),
            )
            .get_parameter_value()
            .bool_value
        )

        self.get_logger().info(f"image_topic : {self.image_topic}")
        self.get_logger().info(f"detections_topic : {self.detections_topic}")

        self.processed_image_publisher = self.create_publisher(
            Image, "processed_image", qos_profile_sensor_data
        )

        self.image_subscription = message_filters.Subscriber(
            self, Image, self.image_topic, qos_profile=qos_profile_sensor_data
        )

        self.detection_subscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            self.detections_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_subscription, self.detection_subscription), 5, 0.01
        )
        self.synchronizer.registerCallback(self.detection_callback)

        self.br = CvBridge()

    def detection_callback(self, image_msg: Image, detections_msg: Detection2DArray):
        print("detect!!!")
        cv_image = self.br.imgmsg_to_cv2(image_msg, "bgr8")

        for detection in detections_msg.detections:
            detection: Detection2D
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            sx = detection.bbox.size_x
            sy = detection.bbox.size_y

            min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
            max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))

            cv2.rectangle(cv_image, min_pt, max_pt, self.COLOR, self.THICKNESS)

            hypo: ObjectHypothesisWithPose
            hypo = detection.results[0]
            label = "{} {:.3f}".format(
                names[int(hypo.hypothesis.class_id)], hypo.hypothesis.score
            )
            pos = (min_pt[0] + self.THICKNESS, max_pt[1] - self.THICKNESS - 1)
            cv2.putText(
                cv_image, label, pos, self.FONT, 0.75, self.COLOR, 1, cv2.LINE_AA
            )

        if self.preview:
            cv2.imshow("Processed Image", cv_image)
            cv2.waitKey(1)

        processed_image_msg = self.br.cv2_to_imgmsg(
            cv_image
        )
        processed_image_msg.header = image_msg.header

        self.processed_image_publisher.publish(processed_image_msg)
        self.get_logger().info(f"{detections_msg}")


def main(args=None):
    rclpy.init(args=args)
    node = VisualizeNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    except Exception as e:
        node.get_logger().error(f"Exception: {e}")

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
