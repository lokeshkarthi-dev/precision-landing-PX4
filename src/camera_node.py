#!/usr/bin/env python3
"""
camera_node.py — Perception layer for Autonomous Precision Landing System.

Subscribes to the downward-facing gimbal camera feed from Gazebo, detects
an ArUco marker using OpenCV, computes normalized pixel errors relative to
the image center, and publishes them so precision_landing.py can act on them.

Published topics:
    /aruco_error_x  (std_msgs/Float32) — horizontal error, range [-1.0, +1.0]
    /aruco_error_y  (std_msgs/Float32) — vertical error,   range [-1.0, +1.0]
    /camera_debug   (sensor_msgs/Image) — annotated debug frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):
    """ROS2 node that detects ArUco markers and publishes alignment errors."""

    def __init__(self):
        super().__init__('aruco_detector')

        # ── CvBridge ────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Subscriber: raw camera feed from Gazebo ──────────────────────────
        self.subscription = self.create_subscription(
            Image,
            '/gimbal_camera',
            self.image_callback,
            10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_x    = self.create_publisher(Float32, '/aruco_error_x', 10)
        self.pub_y    = self.create_publisher(Float32, '/aruco_error_y', 10)
        self.debug_pub = self.create_publisher(Image,  '/camera_debug',  10)

        # ── ArUco detector setup ─────────────────────────────────────────────
        # DICT_4X4_50: 4×4 binary grid, 50 unique marker IDs (0–49).
        # Fast to detect and robust under moderate lighting variation.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector   = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.parameters
        )

        self.get_logger().info('Aruco detector started')

    # ─────────────────────────────────────────────────────────────────────────
    def image_callback(self, msg: Image) -> None:
        """Process one camera frame: detect marker, publish errors."""
        try:
            # ROS2 Image → OpenCV BGR array
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Downsample 2× for speed (e.g. 1280×720 → 640×360)
            frame = frame[::2, ::2]

            H, W = frame.shape[:2]
            cx, cy = W // 2, H // 2          # image centre

            debug_frame = frame.copy()

            # Crosshair at image centre (cyan)
            cv2.line(debug_frame, (0, cy),  (W, cy),  (255, 255, 0), 2)
            cv2.line(debug_frame, (cx, 0),  (cx, H),  (255, 255, 0), 2)

            # Detection requires grayscale input
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

                # Use the first detected marker
                c  = corners[0][0]                    # shape (4, 2)
                mx = int(np.mean(c[:, 0]))            # marker centre x
                my = int(np.mean(c[:, 1]))            # marker centre y

                # Normalised errors — range [-1.0, +1.0]
                # nx > 0 → marker is to the right; ny > 0 → marker is below centre
                nx = (mx - cx) / cx
                ny = (my - cy) / cy

                # Publish errors
                msgx      = Float32(); msgx.data = float(nx); self.pub_x.publish(msgx)
                msgy      = Float32(); msgy.data = float(ny); self.pub_y.publish(msgy)

                # Debug annotations
                cv2.circle(debug_frame, (mx, my), 5, (0, 0, 255), -1)
                cv2.arrowedLine(debug_frame, (cx, cy), (mx, my), (0, 255, 0), 2)
                cv2.putText(
                    debug_frame,
                    f'nx={nx:.2f}, ny={ny:.2f}',
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                print(f'[ARUCO] nx={nx:.2f}, ny={ny:.2f}')
            else:
                print('[ARUCO] Marker not detected')

            # Show live window and publish debug image
            cv2.imshow('Aruco Detection', debug_frame)
            cv2.waitKey(1)

            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')


# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
