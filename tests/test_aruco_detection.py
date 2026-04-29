#!/usr/bin/env python3
"""
tests/test_aruco_detection.py
Smoke test: generates a synthetic ArUco marker image and verifies
camera_node's detection + error calculation logic offline (no ROS2/PX4 needed).

Run with:
    python3 tests/test_aruco_detection.py
"""

import sys
import os
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


def generate_test_frame(marker_id: int = 0, img_size: int = 640, marker_size: int = 100):
    """Create a white image with an ArUco marker placed at a known offset."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

    frame = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

    # Place marker 80px right and 60px below image centre
    cx, cy = img_size // 2, img_size // 2
    offset_x, offset_y = 80, 60
    top_left_x = cx + offset_x - marker_size // 2
    top_left_y = cy + offset_y - marker_size // 2

    frame[top_left_y:top_left_y + marker_size,
          top_left_x:top_left_x + marker_size] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

    return frame, offset_x / cx, offset_y / cy   # frame, expected_nx, expected_ny


def detect_marker(frame):
    """Replicate camera_node detection logic."""
    H, W = frame.shape[:2]
    cx, cy = W // 2, H // 2
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params     = cv2.aruco.DetectorParameters()
    detector   = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return None, None

    c  = corners[0][0]
    mx = int(np.mean(c[:, 0]))
    my = int(np.mean(c[:, 1]))
    nx = (mx - cx) / cx
    ny = (my - cy) / cy
    return nx, ny


def run_tests():
    print('=' * 50)
    print('ArUco Detection Smoke Tests')
    print('=' * 50)

    # Test 1: marker detected and errors match expected values
    frame, exp_nx, exp_ny = generate_test_frame()
    nx, ny = detect_marker(frame)

    assert nx is not None, 'FAIL: marker not detected'
    assert abs(nx - exp_nx) < 0.05, f'FAIL: nx={nx:.3f}, expected≈{exp_nx:.3f}'
    assert abs(ny - exp_ny) < 0.05, f'FAIL: ny={ny:.3f}, expected≈{exp_ny:.3f}'
    print(f'PASS  Marker detected — nx={nx:.3f} (expected {exp_nx:.3f}), '
          f'ny={ny:.3f} (expected {exp_ny:.3f})')

    # Test 2: centred marker gives near-zero errors
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, 0, 100)
    frame2     = np.ones((640, 640, 3), dtype=np.uint8) * 255
    frame2[270:370, 270:370] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
    nx2, ny2   = detect_marker(frame2)

    assert nx2 is not None, 'FAIL: centred marker not detected'
    assert abs(nx2) < 0.1, f'FAIL: centred nx should be near 0, got {nx2:.3f}'
    assert abs(ny2) < 0.1, f'FAIL: centred ny should be near 0, got {ny2:.3f}'
    print(f'PASS  Centred marker — nx={nx2:.3f}, ny={ny2:.3f} (both near 0)')

    # Test 3: blank image returns None
    blank         = np.ones((640, 640, 3), dtype=np.uint8) * 255
    nx3, ny3      = detect_marker(blank)
    assert nx3 is None, f'FAIL: expected no detection on blank frame, got nx={nx3}'
    print('PASS  No false detection on blank frame')

    print('=' * 50)
    print('All tests passed.')


if __name__ == '__main__':
    run_tests()
