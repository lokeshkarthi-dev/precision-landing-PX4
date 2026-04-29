#!/usr/bin/env python3
"""
assets/generate_marker.py
Generates and saves an ArUco marker image for printing.

Usage:
    python3 generate_marker.py              # generates marker ID 0 at 600px
    python3 generate_marker.py --id 5 --size 800
"""

import argparse
import cv2

def main():
    parser = argparse.ArgumentParser(description='Generate an ArUco marker image.')
    parser.add_argument('--id',   type=int, default=0,   help='Marker ID (0–49)')
    parser.add_argument('--size', type=int, default=600, help='Output image size in pixels')
    parser.add_argument('--out',  type=str, default=None, help='Output file path (default: marker_<id>.png)')
    args = parser.parse_args()

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, args.id, args.size)

    out_path = args.out or f'markers/marker_{args.id}.png'
    cv2.imwrite(out_path, marker_img)
    print(f'Saved: {out_path}  (ID={args.id}, size={args.size}px)')
    print('Print at ≥15 cm × 15 cm for reliable detection at 5 m altitude.')

if __name__ == '__main__':
    main()
