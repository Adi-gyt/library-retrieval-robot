#!/usr/bin/env python3
"""
Sim Step 3, Phase A — confirm ArUco detection actually works, using the
static marker image directly. No ROS2, no Gazebo — this isolates "does
the detection code work" from "is the sim wired up correctly," so if
something's broken later, we already know which half is at fault.
"""

import cv2

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(ARUCO_DICT, detector_params)

image = cv2.imread('aruco_marker_0.png')
if image is None:
    raise FileNotFoundError('aruco_marker_0.png not found — run generate_marker.py first')

corners, ids, rejected = detector.detectMarkers(image)

if ids is not None:
    print(f'Detected {len(ids)} marker(s): {ids.flatten().tolist()}')
    print(f'Corner pixel coordinates of marker 0:\n{corners[0]}')

    # draw the detection outline + ID label onto the image, save it, so
    # there's a visual to actually look at, not just printed numbers
    annotated = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
    cv2.imwrite('aruco_marker_0_detected.png', annotated)
    print('Saved aruco_marker_0_detected.png with detection outline drawn on it')
else:
    print('No markers detected — something is wrong with the detection setup')
