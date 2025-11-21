import cv2
import cv2.aruco as aruco
import numpy as np
from collections import defaultdict
import time

# === Load camera calibration ===
filename = "aruco-detection/tutorial_camera_params.yml"
fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("camera_matrix").mat()
dist_coeffs = fs.getNode("distortion_coefficients").mat()
fs.release()

# === Video source ===
video_path = "/home/andrei/erc-software/aruco-detection/VID_20251107_141125768.mp4"
cap = cv2.VideoCapture(video_path)
# === ArUco setup ===
dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
detector = aruco.ArucoDetector(dictionary)
marker_length = 0.05  # meters (5 cm)

# === Pose smoothing + memory ===
ALPHA = 0.6  # smoothing factor (higher = smoother, slower)
prev_tvec = {}
prev_rvec = {}
id_last_seen = defaultdict(lambda: 0)
MEMORY = 5  # remember marker for 5 frames
frame_count = 0

# === FPS tracking ===
prev_time = time.time()
fps = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_count += 1

    # Convert to grayscale for faster/more stable detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    marker_corners, marker_ids, _ = detector.detectMarkers(gray)

    # Corner refinement for more stable pose
    if marker_corners:
        for i in range(len(marker_corners)):
            cv2.cornerSubPix(
                gray,
                marker_corners[i],
                winSize=(3, 3),
                zeroZone=(-1, -1),
                criteria=(
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                ),
            )

    # Draw markers & estimate pose
    if marker_ids is not None:
        aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, marker_length, camera_matrix, dist_coeffs
        )

        for corners, marker_id_arr, rvec, tvec in zip(
            marker_corners, marker_ids, rvecs, tvecs
        ):
            marker_id = int(marker_id_arr[0])
            id_last_seen[marker_id] = frame_count  # remember this marker

            # Smooth pose (exponential moving average)
            if marker_id in prev_tvec:
                tvec = ALPHA * prev_tvec[marker_id] + (1 - ALPHA) * tvec
                rvec = ALPHA * prev_rvec[marker_id] + (1 - ALPHA) * rvec
            prev_tvec[marker_id] = tvec
            prev_rvec[marker_id] = rvec

            # Draw coordinate axes
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            # Draw magenta ID label (top-right above center)
            corners = corners.reshape((4, 2))
            center = corners.mean(axis=0).astype(int)
            text_pos = (center[0] + 40, center[1] - 30)
            text = f"ID {marker_id}"
            # print(marker_id)

            # black shadow for visibility
            cv2.putText(
                frame,
                text,
                (text_pos[0] + 2, text_pos[1] + 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            # magenta text
            cv2.putText(
                frame,
                text,
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

    # Forget old markers not seen for a while
    for mid, last_seen in list(id_last_seen.items()):
        if frame_count - last_seen > MEMORY:
            del id_last_seen[mid]
            prev_tvec.pop(mid, None)
            prev_rvec.pop(mid, None)

    # --- FPS overlay ---
    curr_time = time.time()
    dt = curr_time - prev_time
    fps = 0.9 * fps + 0.1 * (1.0 / dt) if dt > 0 else fps
    prev_time = curr_time
    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    # Display
    cv2.imshow("ArUco Pose Estimation", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
