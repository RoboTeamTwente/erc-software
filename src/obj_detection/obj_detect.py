from ultralytics import YOLO
import cv2

# Load pretrained YOLO (temporary)
model = YOLO("yolov8n.pt")

# Open phone camera stream
cap = cv2.VideoCapture("http://192.168.0.152:8080/video", cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("❌ Cannot open camera stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Frame read failed")
        break

    # Run YOLO inference
    results = model(frame, conf=0.3)

    # Draw results
    annotated_frame = results[0].plot()

    cv2.imshow("YOLO Rock Prototype", annotated_frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
