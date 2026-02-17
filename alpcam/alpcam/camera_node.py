"""ROS 2 node for the Alp stereo camera.

Opens a USB stereo camera that delivers left+right images in a single
side-by-side frame, splits them, and publishes standard stereo image topics.
"""

import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from alpcam.device import find_camera_device

# Supported combined-frame resolutions (width x height) → per-eye is half width.
RESOLUTIONS = {
    "3200x1200": (3200, 1200),
    "2560x960":  (2560, 960),
    "2560x720":  (2560, 720),
    "1280x480":  (1280, 480),
}


def load_camera_info_from_yaml(path: str, frame_id: str) -> CameraInfo | None:
    """Load a CameraInfo message from a YAML file (camera_calibration format).

    Returns None if the file doesn't exist or can't be parsed.
    """
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except (FileNotFoundError, OSError):
        return None
    if data is None:
        return None

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = data.get("image_width", 0)
    msg.height = data.get("image_height", 0)
    msg.distortion_model = data.get("distortion_model", "")

    dm = data.get("distortion_coefficients", {})
    msg.d = dm.get("data", [])

    cm = data.get("camera_matrix", {})
    msg.k = cm.get("data", [0.0] * 9)

    rm = data.get("rectification_matrix", {})
    msg.r = rm.get("data", [0.0] * 9)

    pm = data.get("projection_matrix", {})
    msg.p = pm.get("data", [0.0] * 12)

    return msg


def make_default_camera_info(width: int, height: int, frame_id: str) -> CameraInfo:
    """Return a CameraInfo with dimensions set but no calibration data."""
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    return msg


class AlpCameraNode(Node):
    def __init__(self):
        super().__init__("alpcam_camera")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("device", "")
        self.declare_parameter("resolution", "3200x1200")
        self.declare_parameter("frame_rate", 30)
        self.declare_parameter("frame_id", "alpcam_link")
        self.declare_parameter("left_camera_info_url", "")
        self.declare_parameter("right_camera_info_url", "")
        self.declare_parameter("rotate_180", True)

        self.device_param = self.get_parameter("device").value
        self.resolution = self.get_parameter("resolution").value
        self.frame_rate = self.get_parameter("frame_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.left_info_url = self.get_parameter("left_camera_info_url").value
        self.right_info_url = self.get_parameter("right_camera_info_url").value
        self.rotate_180 = self.get_parameter("rotate_180").value

        # ── Publishers ──────────────────────────────────────────────
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        info_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub_left_img = self.create_publisher(Image, "~/left/image_raw", qos)
        self.pub_right_img = self.create_publisher(Image, "~/right/image_raw", qos)
        self.pub_left_info = self.create_publisher(CameraInfo, "~/left/camera_info", qos)
        self.pub_right_info = self.create_publisher(CameraInfo, "~/right/camera_info", qos)

        self.bridge = CvBridge()
        self.cap = None
        self._frame_count = 0

        # ── Open camera ────────────────────────────────────────────
        self._open_camera()

        # ── Load calibration (if available) ─────────────────────────
        self._load_calibration()

        # ── Timer (capture loop) ────────────────────────────────────
        period = 1.0 / self.frame_rate
        self.timer = self.create_timer(period, self._timer_callback)
        self.get_logger().info(
            f"Streaming at {self.resolution} @ {self.frame_rate} fps "
            f"from {self._device_path}"
        )

    # ── Camera open / close ─────────────────────────────────────────

    def _open_camera(self):
        """Open the V4L2 device and configure resolution + fps."""
        if self.cap is not None:
            self.cap.release()

        # Resolve device path.
        if self.device_param:
            self._device_path = self.device_param
        else:
            self._device_path = find_camera_device()
            if self._device_path is None:
                self.get_logger().error(
                    "Camera not found! Plug it in or set the 'device' parameter."
                )
                raise RuntimeError("Camera not found")

        self.get_logger().info(f"Opening camera at {self._device_path}")
        self.cap = cv2.VideoCapture(self._device_path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open {self._device_path}")
            raise RuntimeError(f"Failed to open {self._device_path}")

        # Set combined resolution.
        if self.resolution not in RESOLUTIONS:
            self.get_logger().warn(
                f"Unknown resolution '{self.resolution}', "
                f"valid options: {list(RESOLUTIONS.keys())}.  Using 3200x1200."
            )
            self.resolution = "3200x1200"

        w, h = RESOLUTIONS[self.resolution]
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

        # Confirm actual settings.
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Camera opened: {actual_w}x{actual_h} @ {actual_fps} fps "
            f"(requested {w}x{h} @ {self.frame_rate})"
        )

        self._eye_width = actual_w // 2
        self._eye_height = actual_h

    # ── Calibration loading ─────────────────────────────────────────

    def _load_calibration(self):
        """Load left/right CameraInfo from YAML files, or use defaults."""
        eye_w = self._eye_width
        eye_h = self._eye_height

        if self.left_info_url:
            info = load_camera_info_from_yaml(self.left_info_url, self.frame_id)
            if info:
                self.left_camera_info = info
                self.get_logger().info(f"Loaded left calibration from {self.left_info_url}")
            else:
                self.left_camera_info = make_default_camera_info(eye_w, eye_h, self.frame_id)
                self.get_logger().warn(f"Could not load left calibration from {self.left_info_url}")
        else:
            self.left_camera_info = make_default_camera_info(eye_w, eye_h, self.frame_id)

        if self.right_info_url:
            info = load_camera_info_from_yaml(self.right_info_url, self.frame_id)
            if info:
                self.right_camera_info = info
                self.get_logger().info(f"Loaded right calibration from {self.right_info_url}")
            else:
                self.right_camera_info = make_default_camera_info(eye_w, eye_h, self.frame_id)
                self.get_logger().warn(f"Could not load right calibration from {self.right_info_url}")
        else:
            self.right_camera_info = make_default_camera_info(eye_w, eye_h, self.frame_id)

    # ── Capture + publish loop ──────────────────────────────────────

    def _timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame", throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()

        if self._frame_count == 0:
            h, w = frame.shape[:2]
            self.get_logger().info(
                f"First frame: {w}x{h} (expected side-by-side → "
                f"{w//2}x{h} per eye)"
            )
        self._frame_count += 1

        # Split combined side-by-side frame into left and right halves.
        mid = frame.shape[1] // 2
        left_img = frame[:, :mid]
        right_img = frame[:, mid:]

        if self.rotate_180:
            # Sensor is upside-down: rotate each eye 180° and swap L/R.
            left_img, right_img = (
                cv2.rotate(right_img, cv2.ROTATE_180),
                cv2.rotate(left_img, cv2.ROTATE_180),
            )

        # Convert to ROS Image messages.
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding="bgr8")
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding="bgr8")

        left_msg.header.stamp = stamp
        left_msg.header.frame_id = self.frame_id
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = self.frame_id

        # Stamp CameraInfo and publish.
        self.left_camera_info.header.stamp = stamp
        self.right_camera_info.header.stamp = stamp

        self.pub_left_img.publish(left_msg)
        self.pub_right_img.publish(right_msg)
        self.pub_left_info.publish(self.left_camera_info)
        self.pub_right_info.publish(self.right_camera_info)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AlpCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
