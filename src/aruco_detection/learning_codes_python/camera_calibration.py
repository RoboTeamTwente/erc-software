import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time

video_path = "https://145.126.44.42:8080/video"
min_captures = 25  # number of valid frames to collect
capture_interval = 5.0  # seconds between snapshots

# === Board parameters ===
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
CHARUCOBOARD_ROWCOUNT = 5
CHARUCOBOARD_COLCOUNT = 5
CHARUCO_BOARD = aruco.CharucoBoard(
    (CHARUCOBOARD_COLCOUNT, CHARUCOBOARD_ROWCOUNT),
    0.025,  # square length in meters
    0.018,  # marker length in meters
    ARUCO_DICT,
)

# === Output folders ===
os.makedirs("aruco-detection", exist_ok=True)
os.makedirs("aruco-detection/calib_images", exist_ok=True)

corners_all, ids_all = [], []
image_size = None
validCaptures = 0

cap = cv2.VideoCapture(video_path)
print("Starting automatic calibration capture...")
print("Press 'q' to quit early.\n")

last_capture_time = 0.0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Failed to read frame from camera.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, CHARUCO_BOARD
        )

        # Only capture every capture_interval seconds
        current_time = time.time()
        if (
            response is not None
            and response > 10
            and (current_time - last_capture_time) >= capture_interval
        ):
            corners_all.append(charuco_corners)
            ids_all.append(charuco_ids)
            validCaptures += 1
            last_capture_time = current_time

            # Save snapshot
            img_path = f"aruco-detection/calib_images/frame_{validCaptures:02d}.jpg"
            cv2.imwrite(img_path, frame)
            print(f"📸 Captured valid frame {validCaptures} → {img_path}")

            if image_size is None:
                image_size = gray.shape[::-1]

    # Display feed for monitoring
    cv2.imshow("Calibration Capture", frame)
    if cv2.waitKey(1) & 0xFF == ord("q") or validCaptures >= min_captures:
        break

cap.release()
cv2.destroyAllWindows()

print(f"\nTotal valid captures: {validCaptures}")

if validCaptures < 3:
    print("Not enough valid frames for calibration.")
    exit()

print("\nCalibrating camera... (this may take a few seconds)")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
    corners_all, ids_all, CHARUCO_BOARD, image_size, None, None
)

print("\n=== Calibration Results ===")
print(f"RMS Reprojection Error: {ret:.6f}")
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs.ravel())

output_file = "aruco-detection/charuco_camera_params.yml"
fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
fs.write("camera_matrix", camera_matrix)
fs.write("distortion_coefficients", dist_coeffs)
fs.release()

print(f"\n✅ Calibration saved to {output_file}")
