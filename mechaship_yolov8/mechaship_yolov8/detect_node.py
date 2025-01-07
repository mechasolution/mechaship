import time
from os.path import join

# OpenCV
import cv2
import numpy as np
from cv_bridge import CvBridge

# ROS 2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle.publisher import LifecyclePublisher
from rclpy.qos import qos_profile_sensor_data

# ROS 2 Messages and Services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# RKNN
from rknnlite.api import RKNNLite

from .rknn_helper import RKNNHelper

# fmt: off
CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis','snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
DEBUG = True
# fmt: on


class DetectNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__(
            "detect_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

    # 디버그 메세지 출력 함수
    def log_debug(self, message: str) -> None:
        if DEBUG:
            self.get_logger().info(message)

    # <on_configure> 노드를 생성할 때 / 변수 선언
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Configuring {self.get_name()}")

        # 파라미터 가져오기
        self.image_topic = (
            self.get_parameter_or("image_topic", "/image_raw/compressed")
            .get_parameter_value()
            .string_value
        )

        self.model = (
            self.get_parameter_or("model_params.model", "yolov8.rknn")
            .get_parameter_value()
            .string_value
        )

        self.img_size = (
            self.get_parameter_or("model_params.img_size", 640)
            .get_parameter_value()
            .integer_value
        )

        _conf = (
            self.get_parameter_or("model_params.conf", 0.5)
            .get_parameter_value()
            .double_value
        )

        _iou = (
            self.get_parameter_or("model_params.iou", 0.8)
            .get_parameter_value()
            .double_value
        )

        self.enable = (
            self.get_parameter_or("model_params.enable", True)
            .get_parameter_value()
            .bool_value
        )

        self.log_debug(f"image_topic : {self.image_topic}")
        self.log_debug(f"model : {self.model}")
        self.log_debug(f"img_size : {self.img_size}")
        self.log_debug(f"conf : {_conf}")
        self.log_debug(f"iou : {_iou}")
        self.log_debug(f"enable : {self.enable}")

        # RKNN 핼퍼 불러오기 (rknn_helper.py)
        self.rknn_helper = RKNNHelper(_conf, _iou, (self.img_size, self.img_size))

        # 객체 인식 결과 Publisher 생성
        self.detection_publisher: LifecyclePublisher = self.create_lifecycle_publisher(
            Detection2DArray, "detections", qos_profile_sensor_data
        )

        # 객체 인식 실행 여부 Subscription 생성
        self.detection_enable_subscription = self.create_subscription(
            Bool,
            "detection_enable",
            self.detection_enable_callback,
            qos_profile_sensor_data,
        )

        # CvBridge 불러오기
        self.br = CvBridge()

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_configure>
        super().on_configure(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # 객체 인식 실행 여부 Subscription callback
    def detection_enable_callback(self, enable_msg: Bool) -> None:
        # self.enable 변수 업데이트
        self.enable = enable_msg.data

    # <on_activate> 노드를 실행할 때 / 시스템 구성
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Activating {self.get_name()}")

        # RKNN Lite 불러오기
        self.rknn_lite = RKNNLite()

        # ONNX model 불러오기
        self.log_debug("--> Load RKNN model")
        model_full_path = join(
            get_package_share_directory(__package__), "model", self.model
        )
        ret = self.rknn_lite.load_rknn(model_full_path)
        if ret != 0:  # 실패할 경우 예외 발생
            raise RuntimeError("Load RKNN model failed!")
        self.log_debug("done")

        # 런타임 환경 초기화
        self.log_debug("--> Init runtime environment")
        ret = self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:  # 실패할 경우 예외 발생
            raise RuntimeError("Init runtime environment failed!")
        self.log_debug("done")

        # 이미지 Subscription 생성
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_activate>
        super().on_activate(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # <on_deactivate> 노드를 중지할 때 / 시스템 정지
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Deactivating {self.get_name()}")

        # RKNN Lite 반환하기
        self.rknn_lite.release()

        # 이미지 Subscription 반환하기
        self.destroy_subscription(self.image_subscription)
        self.image_subscription = None

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_deactivate>
        super().on_deactivate(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # <on_cleanup> 노드를 초기화할 때 / 시스템 초기화
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Cleaning up {self.get_name()}")

        # 객체 인식 결과 Publisher 반환하기
        self.destroy_publisher(self.detection_publisher)

        # 객체 인식 실행 여부 Subscription 반환하기
        self.destroy_subscription(self.detection_enable_subscription)
        self.detection_enable_subscription = None

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_cleanup>
        super().on_cleanup(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # <on_shutdown> 노드를 삭제할 때
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.log_debug(f"Shutting down {self.get_name()}")

        # 모든 요소(publisher, subscriber 등) 상태 전환 >> <on_shutdown>
        super().on_shutdown(state)

        # 상태 전환 결과 반환 (SUCCESS)
        return TransitionCallbackReturn.SUCCESS

    # 이미지 Subscription callback
    def image_callback(self, compressed_image: CompressedImage) -> None:
        # self.enable이 True 일때만 실행
        if not self.enable:
            return

        # 객체 인식 결과 Publisher가 activate 상태일때만 실행
        if not self.detection_publisher.is_activated:
            self.get_logger().warn("Publisher is not activated")
            return

        try:
            # compressed image를 OpenCV 이미지로 변환
            origin_image = self.br.compressed_imgmsg_to_cv2(compressed_image)

            # 비율에 맞춰 이미지 리사이즈
            h, w, _ = origin_image.shape
            scale = self.img_size / max(h, w)
            new_h, new_w = int(h * scale), int(w * scale)
            resized_image = cv2.resize(origin_image, (new_w, new_h))
            padded_image = cv2.copyMakeBorder(
                resized_image,
                top=(self.img_size - new_h) // 2,
                bottom=(self.img_size - new_h) - (self.img_size - new_h) // 2,
                left=(self.img_size - new_w) // 2,
                right=(self.img_size - new_w) - (self.img_size - new_w) // 2,
                borderType=cv2.BORDER_CONSTANT,
                value=(0, 0, 0),
            )

            # RKNN에 사용할 수 있는 구조로 변경 (4dim)
            input_image = np.expand_dims(padded_image, 0)

            # 객체 탐지 실행
            self.log_debug("--> Running model")
            start_time = time.time()
            outputs = self.rknn_lite.inference(inputs=[input_image])
            finish_time = time.time()

            boxes, classes, scores = self.rknn_helper.post_process(outputs)

            # 탐지된 객체가 없을 경우 종료
            if boxes is None or classes is None or scores is None:
                return

            self.log_debug(f"Inference FPS: {1 / (finish_time - start_time)}")
            self.log_debug(
                f"Results: {list(map(lambda x: CLASSES[x], classes)), scores}"
            )

            # Detection2DArray에 탐지 결과 담기
            detections_msg = Detection2DArray()
            detections_msg.header = compressed_image.header
            for box, class_id, score in zip(boxes, classes, scores):
                detection = Detection2D()
                detection.header = compressed_image.header

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(class_id))
                hypothesis.hypothesis.score = float(score)
                detection.results = [hypothesis]

                x1, y1, x2, y2 = box
                x = int((x1 + x2) / 2)
                y = int((y1 + y2) / 2)
                w = int(x2 - x1)
                h = int(y2 - y1)

                detection.bbox.center.position.x = float(x)
                detection.bbox.center.position.y = float(y)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                detections_msg.detections.append(detection)

            # Detection2DArray Publish
            self.detection_publisher.publish(detections_msg)

        except Exception as e:
            self.get_logger().error(f"Exception in image_callback: {e}")


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
