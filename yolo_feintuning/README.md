It integrates dataset handling, model training/validation/inference utilities, and control/following ROS2 nodes.
Develop an object detector that allows the robot to follow a specific object in real-time using YOLO.
This feature will use computer vision to detect and track the object and integrate it with the robot's motion control system.


<!-- 
- Real-time object detection using YOLOv8
- Transfer learning implementation for classification from custom balls 
- Support for three different ball classes
- Camera-based perception pipeline
-->

<!-- This package provides:
-  yolov8n über Transfer Learning trainieren -> Transfer Learning Dokument velinken 
-  feintunede Modelle zsm vergleichen -> GIFs
-  feintunded Modelle VS vortrainierte Modelle
-  ROS2bag -> GIFs erstellen -->
-  
# YOLOv8 Fine-Tuning for Ball Detection

This package contains the scripts and resources used to fine-tune a **YOLOv8n** model for detecting three specific ball classes: `Baseball`, `Tennisball`, and `Football`. We used the **Ultralytics framework** to apply transfer learning, adapting a pre-trained model to our custom dataset. The resulting model is the core of the computer vision system in the `yolo_ros2_interaction` package.

- [YOLOv8 Fine-Tuning for Ball Detection](#yolov8-fine-tuning-for-ball-detection)
  - [Results](#results)
  - [Methodology](#methodology)
    - [1. Dataset Preparation](#1-dataset-preparation)
    - [2. Model Training](#2-model-training)
    - [3. Validation](#3-validation)
  - [Usage](#usage)
  - [Package Structure](#package-structure)


## Results

The fine-tuning process yielded a significant improvement in detection accuracy. Our specialized [`yolov8n-finetuned`](yolo_feintuning\results\train\feinTuned_yolov8n_75_epochs\weights\best.pt) model  outperforms all stock YOLOv8 models on our specific task.

<p align="center">
  <div style="display: inline-block; margin-right: 0px;">
    <img src="results\gifs\BoxF1_curve.gif" width="500px">
  </div>
  <div style="display: inline-block; margin-left: 0px;">
    <img src="results\gifs\confusion_matrix_normalized.gif" width="500px">
  </div>
</p>
<p align="center">
  <div style="display: inline-block; margin-right: 0px;">
    <img src="results\gifs\results.gif" width="1000px">
  </div>

-----

## Methodology

Our workflow is structured into three main stages: dataset preparation, model training, and a final validation phase.

### 1\. Dataset Preparation

The model's performance relies on a specialized dataset created for this task, which is hosted on [Roboflow](https://app.roboflow.com/robotik-7goue/balldetector-pgfsi/5).

  * **Training Set**: Comprises 520 base images, with 176 captured directly from the robot's camera to match the deployment environment. This set was expanded to 1,560 images through augmentation:
      * Horizontal Flip
      * Brightness adjustment (±25%)
      * Blur (up to 0.5px)
      * Noise injection (up to 2%)
  * **Validation Set**: Consists of 120 images taken exclusively from the robot's perspective to ensure a realistic evaluation of the model's performance in its operational context.

### 2\. Model Training

  The fine-tuning process was initiated using the [`yolo_train.py`](yolo_feintuning\scripts\yolo_predict.py) script.

  * **Base Model**: We selected the [`yolov8n.pt`](yolo_feintuning\coco_pretrained_models\yolov8n.pt) model, pre-trained on the COCO dataset, as our starting point.
  * **Process**: Multiple training runs were conducted with varying epoch counts (e.g., 50, 75, 100, 150) to identify the checkpoint with the best performance before overfitting occurred.

### 3\. Validation

Systematic evaluation was performed using the [`yolo_valid.py`](yolo_feintuning\scripts\yolo_valid.py) script. This script was designed to benchmark our fine-tuned models against the original, pre-trained stock models. This comparison is essential to quantify the improvement achieved through fine-tuning.

-----


## Usage

The scripts are designed for a sequential workflow:

1.  **Prepare the dataset** and place it in the `dataset` folder.
2.  **Start training** by running the training script:
    ```bash
    python yolo_train.py
    ```
3.  **Validate the models** to compare performance:
    ```bash
    python yolo_valid.py
    ```
4.  **Test predictions** on new images with the best model:
    ```bash
    python yolo_predict.py
    ```

All outputs from these scripts are saved in the `results` directory.

-----

## Package Structure

```bash
└── 📁yolo_feintuning
    ├── 📁dataset
    │   ├── data_coco.yaml          # YAML for COCO model comparison
    │   └── data.yaml               # YAML for custom model training
    ├── 📁results
    │   ├── 📁predict               # Saved inference results
    │   ├── 📁train                 # Saved training results
    │   └── 📁val                   # Saved validation results
    ├── README.md                   # This file
    ├── yolo_predict.py
    ├── yolo_train.py
    └── yolo_valid.py
```