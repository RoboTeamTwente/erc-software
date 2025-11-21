import cv2
import numpy as np
import os

# Create directories if they don't exist
os.makedirs("aruco-detection/dataset/aruco_labels", exist_ok=True)
os.makedirs("aruco-detection/dataset/aruco_labels_with_bg", exist_ok=True)

# Generate 1000 ArUco
for i in range(1000):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, id=i, sidePixels=200)
    cv2.imwrite(f"aruco-detection/dataset/aruco_labels/aruco_{i:03d}.png", marker_img)

    bg = np.random.randint(0, 255, (300, 300), dtype=np.uint8)
    x, y = 50, 50
    bg[y : y + 200, x : x + 200] = marker_img
    cv2.imwrite(f"aruco-detection/dataset/aruco_labels_with_bg/aruco_{i:03d}.png", bg)

    if i % 100 == 0:
        print(f"Generated {i} ArUco markers")
