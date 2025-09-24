# source: https://docs.ultralytics.com/modes/val/
from ultralytics import YOLO
import os

model_paths = {
    "epoch_10": ("results/train/feinTuned_yolov8n_10_epochs/weights/best.pt", "custom"),
    "epoch_25": ("results/train/feinTuned_yolov8n_25_epochs/weights/best.pt", "custom"),
    "epoch_50": ("results/train/feinTuned_yolov8n_50_epochs/weights/best.pt", "custom"),
    "epoch_75": ("results/train/feinTuned_yolov8n_75_epochs/weights/best.pt", "custom"),
    "epoch_100": ("results/train/feinTuned_yolov8n_100_epochs/weights/best.pt", "custom"),
    "epoch_150": ("results/train/feinTuned_yolov8n_150_epochs/weights/best.pt", "custom"),
    "epoch_200": ("results/train/feinTuned_yolov8n_200_epochs/weights/best.pt", "custom"),
    # "n_coco_pretrained": ("yolov8n.pt", "coco"),
    # "s_coco_pretrained": ("yolov8s.pt", "coco"),
    # "m_coco_pretrained": ("yolov8m.pt", "coco"),
    # "l_coco_pretrained": ("yolov8l.pt", "coco"),
    # "x_coco_pretrained": ("yolov8x.pt", "coco"),
}

feinTunded_val_dataset = "../dataset/data.yaml"
pretrained_val_dataset = "../dataset/data_coco.yaml"

cache_file = "./dataset/labels.cache"

for name, (path, dtype) in model_paths.items():
    model = YOLO(path)

    if os.path.exists(cache_file):
        os.remove(cache_file)

    if dtype == "custom":
        dataset = feinTunded_val_dataset
    else:
        dataset = pretrained_val_dataset

    model.val(
        data = dataset,
        save = True,
        project = "../results/val",
        name = name,
        exist_ok = True
    )