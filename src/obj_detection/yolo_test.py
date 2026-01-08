from ultralytics import YOLO

model = YOLO("yolo11n.pt")
model.predict(
    # source="https://192.168.0.152:8080/video",
    source=0,
    imgsz=640,
    conf=0.15,
    iou=0.75,
    show=True,
)
