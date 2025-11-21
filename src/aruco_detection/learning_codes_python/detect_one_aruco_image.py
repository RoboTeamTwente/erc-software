import cv2
import cv2.aruco as aruco

input_image = cv2.imread("aruco-detection/images/real-world-tags.jpg")

marker_corners = []
marker_ids = []
rejected_candidates = []

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
detector_params = aruco.DetectorParameters()

detector = aruco.ArucoDetector(dictionary, detector_params)

marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(input_image)

if marker_ids is not None:
    detected_image = aruco.drawDetectedMarkers(
        input_image.copy(), marker_corners, marker_ids
    )
    cv2.imshow("Detected ArUco Markers", detected_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows
else:
    print("No markers detected.")
