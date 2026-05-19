#!/usr/bin/env python3

import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


class RespawnManagerNode(Node):
    def __init__(self):
        super().__init__("respawn_manager")

        self.declare_parameter("world_name", "empty")
        self.declare_parameter("model_name", "mechaship")
        self.declare_parameter("model_uri", "model://mechaship")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("reset_time_threshold", 0.5)
        self.declare_parameter("reset_settle_delay", 0.3)
        self.declare_parameter("respawn_delay", 0.5)
        self.declare_parameter("respawn_cooldown", 2.0)

        self._last_time = None
        self._last_respawn_wall_time = 0.0
        self._respawning = False

        self.create_subscription(Clock, "/clock", self._clock_callback, 10)
        self.create_service(Trigger, "~/respawn", self._respawn_service_callback)

    def _clock_callback(self, msg):
        current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self._last_time is None:
            self._last_time = current_time
            return

        threshold = self.get_parameter("reset_time_threshold").value
        if current_time + threshold < self._last_time:
            self.get_logger().info("Gazebo reset detected. Respawning model.")
            self._start_respawn()

        self._last_time = current_time

    def _respawn_service_callback(self, request, response):
        del request
        started = self._start_respawn()
        response.success = started
        response.message = (
            "Respawn started." if started else "Respawn is already running."
        )
        return response

    def _start_respawn(self):
        now = time.monotonic()
        cooldown = self.get_parameter("respawn_cooldown").value
        if self._respawning or now - self._last_respawn_wall_time < cooldown:
            return False

        self._respawning = True
        self._last_respawn_wall_time = now
        threading.Thread(target=self._respawn_model, daemon=True).start()
        return True

    def _respawn_model(self):
        try:
            time.sleep(self.get_parameter("reset_settle_delay").value)
            self._remove_model()
            time.sleep(self.get_parameter("respawn_delay").value)
            self._create_model()
        finally:
            self._respawning = False

    def _remove_model(self):
        world_name = self.get_parameter("world_name").value
        model_name = self.get_parameter("model_name").value

        request = f'name: "{model_name}", type: MODEL'
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{world_name}/remove",
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            request,
        ]

        result = self._call_gz_service(command)
        if not result:
            self.get_logger().warn(
                f"Model remove request failed or returned false: {model_name}"
            )

    def _create_model(self):
        world_name = self.get_parameter("world_name").value
        model_name = self.get_parameter("model_name").value
        model_uri = self.get_parameter("model_uri").value
        x = self.get_parameter("x").value
        y = self.get_parameter("y").value
        z = self.get_parameter("z").value
        yaw = self.get_parameter("yaw").value

        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        request = (
            f'sdf_filename: "{model_uri}", '
            f'name: "{model_name}", '
            f"pose: {{"
            f"position: {{x: {x}, y: {y}, z: {z}}}, "
            f"orientation: {{z: {qz}, w: {qw}}}"
            f"}}"
        )

        command = [
            "gz",
            "service",
            "-s",
            f"/world/{world_name}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            request,
        ]

        result = self._call_gz_service(command)
        if not result:
            self.get_logger().error(
                f"Model create request failed or returned false: {model_name}"
            )
        else:
            self.get_logger().info(f"Model respawn requested: {model_name}")

    def _call_gz_service(self, command):
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode != 0:
            self.get_logger().warn(result.stderr.strip())
            return False

        output = f"{result.stdout}\n{result.stderr}".lower()
        if "data: true" in output or "success: true" in output:
            return True

        self.get_logger().warn(output.strip())
        return False


def main(args=None):
    rclpy.init(args=args)
    node = RespawnManagerNode()

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
