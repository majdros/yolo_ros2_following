# source: https://docs.ultralytics.com/modes/predict/
from ultralytics import YOLO

yolo_version = "feinTuned_yolov8m_100_epochs"
yolo_model = f"results/train/{yolo_version}/weights/best.pt"

model = YOLO(yolo_model, task="detect")
preds = model.predict(
                    source="dataset/valid/images",
                    imgsz=640,
                    conf=0.25,
                    project = f"results/predict/{yolo_version}",
                    save=True
                    )

