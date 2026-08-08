#!/usr/bin/env python3
"""
Sim Step 3, Phase A — generate a single ArUco marker image.

Marker ID 0, from the 4x4_50 dictionary (a common, low-clutter choice —
4x4 grid, 50 possible IDs, easy to detect reliably at small sizes).
This PNG gets used two ways:
  1. Right now, as a static test image to confirm detection code works.
  2. Next, as a texture applied to a flat plane in the Gazebo world, so
     the simulated D405 can actually see a real marker.
"""

import cv2

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_ID = 0
MARKER_SIZE_PX = 400  # pixels; scaled physically later when placed in sim

marker_image = cv2.aruco.generateImageMarker(ARUCO_DICT, MARKER_ID, MARKER_SIZE_PX)

# ArUco detection needs a blank "quiet zone" around the marker to first
# locate its border against the background — without this, the detector
# has nothing to lock onto even though the marker itself is valid.
BORDER_PX = 50
marker_image = cv2.copyMakeBorder(
    marker_image, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
    cv2.BORDER_CONSTANT, value=255  # white
)

cv2.imwrite('aruco_marker_0.png', marker_image)
print(f'Wrote aruco_marker_0.png ({MARKER_SIZE_PX}x{MARKER_SIZE_PX}px marker + {BORDER_PX}px white border, ID {MARKER_ID}, DICT_4X4_50)')
