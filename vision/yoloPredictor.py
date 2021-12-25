from yolov5 import YOLOv5
import cv2
import time
import os
import torch

# from pilot import coPilot
import numpy as np

import sys

model_path = "yolov5/weights/up-downrect-batch49.pt"  # it automatically downloads yolov5s model to given path
device = "cuda"  # or "cpu"
yolov5 = YOLOv5(model_path, device)

frame = cv2.imread("orta6.jpg")
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
results = yolov5.predict(frame)

results = yolov5.predict(frame)
predictions = results.pred[0]
boxes = predictions[:, :4]
scores = predictions[:, 4]
categories = predictions[:, 5]

results.show()
