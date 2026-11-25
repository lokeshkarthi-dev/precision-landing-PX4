#!/usr/bin/env python3
"""
camera_node.py - Perception layer (v2) for the Autonomous Precision Landing System.

What it does
------------
1. Subscribes to the downward-facing mono camera image (bridged from Gazebo).
2. Finds the target ArUco marker with OpenCV.
3. Publishes ONE message with the marker's offset from the image centre.

Published topics
----------------
/aruco_error   (geometry_msgs/PointStamped)
    point.x = nx : horizontal error, + means marker is to the RIGHT of centre
    point.y = ny : vertical error,   + means marker is BELOW centre
    point.z = 0  : unused
    header.stamp = timestamp of the camera frame (so latency can be measured)
    Both errors are divided by HALF THE IMAGE WIDTH, so 1.0 means the same
    angle in both directions. (v1 divided ny by half the height, which made
    vertical errors look 1.78x bigger than horizontal ones on a 16:9 image.)
    Nothing is published when the marker is not seen.

/camera_debug  (sensor_msgs/Image)
    Annotated frame (optional, see parameter publish_debug_image).

All settings are ROS 2 parameters, loaded from config/params.yaml:
    python3 src/camera_node.py --ros-args --params-file config/params.yaml
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ArucoDetector(Node):
    """Detects one ArUco marker per frame and publishes its normalised offset."""

    def __init__(self) -> None:
        super().__init__('aruco_detector')

        # ---- Parameters (defaults here, real values live in params.yaml) ----
        self.declare_parameter('image_topic', '/downcam/image_raw')
        self.declare_parameter('error_topic', '/aruco_error')
        self.declare_parameter('debug_topic', '/camera_debug')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_marker_id', 0)       # -1 = use any marker
        self.declare_parameter('downscale', 1)              # 1 = full size, 2 = half
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_publish_every_n', 3)  # publish 1 of N frames
        self.declare_parameter('show_window', False)        # needs a display
        self.declare_parameter('log_period_s', 1.0)

        p = self.get_parameter
        self.target_id = int(p('target_marker_id').value)
        self.downscale = max(1, int(p('downscale').value))
        self.publish_debug = bool(p('publish_debug_image').value)
        self.debug_every_n = max(1, int(p('debug_publish_every_n').value))
        self.show_window = bool(p('show_window').value)
        self.log_period = float(p('log_period_s').value)

        # ---- ROS I/O ---------------------------------------------------------
        self.bridge = CvBridge()
        # Sensor QoS = "best effort, keep only the newest frame". This avoids
        # building up a queue of old images when processing is slower than the camera.
        self.create_subscription(
            Image, p('image_topic').value, self._on_image, qos_profile_sensor_data
        )
        self.pub_error = self.create_publisher(PointStamped, p('error_topic').value, 10)
        self.pub_debug = self.create_publisher(Image, p('debug_topic').value, 5)

        # ---- ArUco detector --------------------------------------------------
        dict_name = str(p('aruco_dictionary').value)
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        params = cv2.aruco.DetectorParameters()
        # Sub-pixel corners give a steadier centre estimate (less jitter).
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self._frame_count = 0
        self.get_logger().info(
            f'Aruco detector started (dictionary={dict_name}, target_id={self.target_id}, '
            f'image_topic={p("image_topic").value})'
        )

    # ------------------------------------------------------------------------
    def _pick_marker(self, corners, ids):
        """Return the index of the marker to follow, or None."""
        if ids is None or len(ids) == 0:
            return None
        flat_ids = ids.flatten()
        if self.target_id >= 0:
            matches = np.where(flat_ids == self.target_id)[0]
            return int(matches[0]) if len(matches) else None
        # target_id < 0: follow the marker that appears largest in the image
        areas = [cv2.contourArea(c.reshape(-1, 2).astype(np.float32)) for c in corners]
        return int(np.argmax(areas))

    # ------------------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge error: {exc}', throttle_duration_sec=5.0)
            return

        # Optional shrink for speed. INTER_AREA keeps the marker edges clean
        # (plain slicing like frame[::2, ::2] throws pixels away and adds aliasing).
        if self.downscale > 1:
            s = 1.0 / self.downscale
            frame = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)

        h, w = frame.shape[:2]
        cx, cy = w / 2.0, h / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        idx = self._pick_marker(corners, ids)

        nx = ny = None
        mx = my = None
        if idx is not None:
            pts = corners[idx].reshape(4, 2)
            mx = float(np.mean(pts[:, 0]))
            my = float(np.mean(pts[:, 1]))
            # Both axes divided by half the WIDTH -> same scale in x and y.
            nx = (mx - cx) / cx
            ny = (my - cy) / cx

            out = PointStamped()
            out.header = msg.header          # keep the camera timestamp
            out.point.x = float(nx)
            out.point.y = float(ny)
            self.pub_error.publish(out)
            self.get_logger().info(
                f'marker {int(ids.flatten()[idx])}: nx={nx:+.2f} ny={ny:+.2f}',
                throttle_duration_sec=self.log_period,
            )
        else:
            self.get_logger().info(
                'marker not detected', throttle_duration_sec=self.log_period
            )

        self._frame_count += 1
        want_debug = self.publish_debug and (self._frame_count % self.debug_every_n == 0)
        if want_debug or self.show_window:
            self._draw_and_publish(frame, corners, ids, idx, (cx, cy), (mx, my), (nx, ny), msg, want_debug)

    # ------------------------------------------------------------------------
    def _draw_and_publish(self, frame, corners, ids, idx, centre, marker, err, msg, publish):
        """Draw the crosshair, marker and error arrow. Only runs when needed."""
        cx, cy = int(centre[0]), int(centre[1])
        h, w = frame.shape[:2]
        dbg = frame.copy()
        cv2.line(dbg, (0, cy), (w, cy), (255, 255, 0), 1)
        cv2.line(dbg, (cx, 0), (cx, h), (255, 255, 0), 1)

        if idx is not None:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
            mx, my = int(marker[0]), int(marker[1])
            cv2.circle(dbg, (mx, my), 4, (0, 0, 255), -1)
            cv2.arrowedLine(dbg, (cx, cy), (mx, my), (0, 255, 0), 2)
            cv2.putText(dbg, f'nx={err[0]:+.2f} ny={err[1]:+.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(dbg, 'NO MARKER', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if self.show_window:
            cv2.imshow('Aruco Detection', dbg)
            cv2.waitKey(1)
        if publish:
            out = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            out.header = msg.header
            self.pub_debug.publish(out)


# ----------------------------------------------------------------------------
def main() -> None:
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
