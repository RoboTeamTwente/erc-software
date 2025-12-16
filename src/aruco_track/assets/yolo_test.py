import cv2
import torch
from ultralytics import YOLO

# === CONFIG ===
VIDEO_URL = "https://145.126.44.42:8080/video"  # your live feed
MODEL_PATH = "yolo11n-seg.pt"  # pretrained segmentation model

# === Load YOLOv11 segmentation model ===
device = 0 if torch.cuda.is_available() else "cpu"
print("Using device:", device)

model = YOLO(MODEL_PATH)

# === Open video stream ===
cap = cv2.VideoCapture(VIDEO_URL)
if not cap.isOpened():
    raise RuntimeError("ERROR: Could not open video stream. Check URL or connection.")

print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Stream ended or failed.")
        break

    # === Run segmentation inference ===
    results = model.predict(source=frame, device=device, conf=0.4, verbose=False)

    # === Annotated visualization (seg masks, boxes, labels) ===
    annotated = results[0].plot()  # YOLO handles the mask overlay + boxes + labels

    # === Display in OpenCV window ===
    cv2.imshow("YOLOv11 Instance Segmentation", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
