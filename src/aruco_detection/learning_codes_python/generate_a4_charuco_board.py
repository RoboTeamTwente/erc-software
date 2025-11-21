import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_5X5_250
SQUARES_VERTICALLY = 7
SQUARES_HORIZONTALLY = 5

# --- physical parameters (mm) ---
SQUARE_LENGTH_MM = 30
MARKER_LENGTH_MM = 15
MARGIN_MM = 10
DPI = 300

# --- conversions ---
mm_to_px = DPI / 25.4
SQUARE_LENGTH = int(SQUARE_LENGTH_MM * mm_to_px)
MARKER_LENGTH = int(MARKER_LENGTH_MM * mm_to_px)
MARGIN_PX = int(MARGIN_MM * mm_to_px)

# --- A4 size in pixels (portrait) ---
IMG_SIZE = (3508, 2480)
OUTPUT_NAME = "ChArUco_A4_300DPI.png"

dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(
    (SQUARES_HORIZONTALLY, SQUARES_VERTICALLY),
    SQUARE_LENGTH,
    MARKER_LENGTH,
    dictionary,
)

img = board.generateImage(IMG_SIZE, marginSize=MARGIN_PX)
cv2.imwrite(OUTPUT_NAME, img)
print(f"Saved: {OUTPUT_NAME}")
