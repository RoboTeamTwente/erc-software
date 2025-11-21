import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time

# Charuco Board Dimensions and Dictionary
# squaresX = 5
# squaresY = 7
# squareLength = 30
# markerLength = 15

dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)

# Charuco parameters

charuco_params = aruco.CharucoParameters()  # uses default params for now
charuco_params.minmarkers = 3

# Detector paramaeters

detector_params = aruco.DetectorParameters()  # defaults again
detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
detector_params.cornerRefinementWinSize = 8
detector_params.cornerRefinementMinAccuracy = 0.02

# Refine parameters

refine_params = aruco.RefineParameters()  # defaults again
# insert parameters here if you need them

# Board itself
board = aruco.CharucoBoard(
    (5, 7), squareLength=30, markerLength=15, dictionary=dictionary
)

detector = aruco.CharucoDetector(
    board,
    charucoParams=charuco_params,
    detectorParams=detector_params,
    refineParams=refine_params,
)
