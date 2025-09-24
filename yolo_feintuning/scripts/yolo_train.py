# source: https://docs.ultralytics.com/modes/train/
from ultralytics import YOLO

model = YOLO("./yolov8n.pt")
epochs = (10, 25, 50, 75, 100, 150, 200)

for epoch in epochs:
    results = model.train(
                        data = "./dataset/data.yaml",
                        epochs = epoch,
                        batch = 8,
                        imgsz = 640,
                        device = 0,
                        project = "results/train",
                        # patience = 20,
                        optimizer = 'adamW',
                        name = f'feinTuned_yolov8n_{epoch}_epochs'
                        )
