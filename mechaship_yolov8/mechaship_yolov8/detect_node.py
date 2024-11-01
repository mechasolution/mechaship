from os.path import join

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle.publisher import LifecyclePublisher
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
from cv_bridge import CvBridge

from rknnlite.api import RKNNLite

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import time
from .rknn_helper import RKNNHelper

CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis','snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

class DetectNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__(
            "detect_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.image_topic = (
            self.get_parameter_or(
                "image_topic",
                Parameter("image_topic", Parameter.Type.STRING, "/image_raw"),
            )
            .get_parameter_value()
            .string_value
        )

        self.model = (
            self.get_parameter_or(
                "model_params.model",
                Parameter("model", Parameter.Type.STRING, "yolov8.rknn"),
            )
            .get_parameter_value()
            .string_value
        )

        _img_size = (
            self.get_parameter_or(
                "model_params.img_size",
                Parameter("img_size", Parameter.Type.INTEGER, 640),
            )
            .get_parameter_value()
            .integer_value
        )

        _conf = (
            self.get_parameter_or(
                "model_params.conf",
                Parameter("conf", Parameter.Type.DOUBLE, 0.5),
            )
            .get_parameter_value()
            .double_value
        )

        _iou = (
            self.get_parameter_or(
                "model_params.iou",
                Parameter("iou", Parameter.Type.DOUBLE, 0.4),
            )
            .get_parameter_value()
            .double_value
        )

        self.enable = (
            self.get_parameter_or(
                "model_params.enable",
                Parameter("enable", Parameter.Type.BOOL, True),
            )
            .get_parameter_value()
            .bool_value
        )
        self.rknn_helper = RKNNHelper(_conf, _iou, (_img_size,_img_size))

        self.get_logger().info(f"image_topic : {self.image_topic}")
        self.get_logger().info(f"model : {self.model}")
        self.get_logger().info(f"img_size : {_img_size}")
        self.get_logger().info(f"conf : {_conf}")
        self.get_logger().info(f"iou : {_iou}")
        self.get_logger().info(f"enable : {self.enable}")

        self.detection_publisher: LifecyclePublisher = self.create_lifecycle_publisher(
            Detection2DArray, "detections", qos_profile_sensor_data
        )
        self.detection_enable_srv = self.create_service(
            SetBool, "enable", self.enable_callback
        )
        self.br = CvBridge()

        super().on_configure(state)

        return TransitionCallbackReturn.SUCCESS

    def enable_callback(self, request, response):
        self.enable = request.data
        response.success = True
        return response

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        # Create RKNN object
        self.rknn_lite = RKNNLite()

        # Load ONNX model
        self.get_logger().info("--> Load RKNN model")
        model_full_path = join(
            get_package_share_directory(__package__), "model", self.model
        )
        ret = self.rknn_lite.load_rknn(model_full_path)
        if ret != 0:
            self.get_logger().info("Load RKNN model failed")
            exit(ret)
        self.get_logger().info("done")

         # Init runtime environment
        self.get_logger().info('--> Init runtime environment')
        ret = self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            self.get_logger().info('Init runtime environment failed!')
            exit(ret)
        self.get_logger().info('done')

        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        self.rknn_lite.release()

        self.destroy_subscription(self.image_subscription)
        self.image_subscription = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        self.destroy_publisher(self.detection_publisher)

        super().on_cleanup(state)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Shutting down {self.get_name()}")

        super().on_shutdown(state)

        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg: Image) -> None:
        if self.enable:
            origin_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            origin_image = cv2.imread('/home/ubuntu/rknn_model_zoo/examples/yolov8/model/bus.jpg')
            input_image = cv2.resize(origin_image, (640, 640))
            input_image = np.expand_dims(input_image, 0) # RKNN expects 4dim image

            self.get_logger().info('--> Running model')
            start_time = time.time()
            outputs = self.rknn_lite.inference(inputs=[input_image])
            finish_time = time.time()

            boxes, classes, scores = self.rknn_helper.post_process(outputs)
            print(boxes, list(map(lambda x: CLASSES[x], classes)), scores)
            self.get_logger().info(f"Inference FPS: {1 / (finish_time - start_time)}")

            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            for box, class_id, score in zip(boxes, classes, scores):
                detection = Detection2D()
                detection.header = msg.header

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(class_id))
                hypothesis.hypothesis.score = float(score)
                detection.results = [hypothesis]

                x1, y1, x2, y2 = box
                x = int((x1+x2)/2)
                y = int((y1+y2)/2)
                w = int(x2-x1)
                h = int(y2-y1)

                detection.bbox.center.position.x = float(x)
                detection.bbox.center.position.y = float(y)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                # detection.id = self.yolo.names[int(np.argmax(classes_scores))]

                detections_msg.detections.append(detection)

            if not self.detection_publisher.is_activated:
                self.get_logger().warn("Publisher is not activated")

            self.detection_publisher.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()
    node.trigger_configure()
    node.trigger_activate()

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
