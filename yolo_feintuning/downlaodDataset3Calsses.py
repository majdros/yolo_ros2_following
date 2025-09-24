from roboflow import Roboflow

rf = Roboflow(api_key="**************")
project = rf.workspace("robotik-7goue").project("balldetector-pgfsi")
version = project.version(5)
dataset = version.download("yolov8")