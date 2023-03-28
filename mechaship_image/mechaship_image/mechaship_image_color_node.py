import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ColorCodeError(Exception):
    def __str__(self):
        return "color code error!"


class MechashipImageColor(Node):
    def __init__(self):
        super().__init__("mechaship_image_color_node")

        self.image_subscription = self.create_subscription(
            Image, "Image", self.listener_callback, qos_profile_sensor_data
        )
        self.image_subscription  # prevent unused variable warning
        self.br = CvBridge()

        self.COLOR_BGR = 0
        self.COLOR_HSV = 1

    def get_point_color(self, image, point_x, point_y, color_code):
        point = image[point_y][point_x]
        if color_code == self.COLOR_BGR:
            text = f"B : {point[0]}, G : {point[1]}, R : {point[2]}"
            cv2.circle(image, (point_x, point_y), 3, (0, 0, 200), -1)
        elif color_code == self.COLOR_HSV:
            text = f"H : {point[0]}, S : {point[1]}, V : {point[2]}"
            cv2.circle(image, (point_x, point_y), 3, (255, 255, 255), -1)
        else:
            raise ColorCodeError()

        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        self.get_logger().info(text)

        return image

    def click_event(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info("CLICKED!!!")
            image = self.get_point_color(
                params["image"].copy(), x, y, params["color_code"]
            )
            self.image_show(image, params["color_code"])
            cv2.waitKey()

    def set_mouse_callback(self, image, color_code):
        if color_code == self.COLOR_BGR:
            title = "BGR image"
        elif color_code == self.COLOR_HSV:
            title = "HSV image"
        else:
            raise ColorCodeError()

        params = {
            "color_code": color_code,
            "image": image,
        }
        cv2.setMouseCallback(title, self.click_event, params)

    def image_show(self, image, color_code):
        if color_code == self.COLOR_BGR:
            title = "BGR image"
        elif color_code == self.COLOR_HSV:
            title = "HSV image"
        else:
            raise ColorCodeError()

        cv2.imshow(title, image)
        cv2.waitKey(1)

    def listener_callback(self, data):
        origin_image = self.br.imgmsg_to_cv2(data, "bgr8")
        h, w, _ = origin_image.shape
        center_x = int(w / 2)
        center_y = int(h / 2)

        # BGR 이미지 중간 좌표 색상 출력
        bgr_img = self.get_point_color(
            origin_image.copy(), center_x, center_y, self.COLOR_BGR
        )
        if len(bgr_img):
            self.image_show(bgr_img, self.COLOR_BGR)
            self.set_mouse_callback(origin_image, self.COLOR_BGR)

        # HSV 이미지 중간 좌표 색상 출력
        origin_hsv_image = cv2.cvtColor(origin_image.copy(), cv2.COLOR_BGR2HSV)
        hsv_img = self.get_point_color(
            origin_hsv_image.copy(), center_x, center_y, self.COLOR_HSV
        )
        if len(hsv_img):
            self.image_show(hsv_img, self.COLOR_HSV)
            self.set_mouse_callback(origin_hsv_image, self.COLOR_HSV)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipImageColor()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
