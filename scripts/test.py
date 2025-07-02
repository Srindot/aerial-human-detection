import numpy
import torch
from ultralytics import YOLO

print("✅ NumPy version:", numpy.__version__)
print("✅ Torch version:", torch.__version__)
print("✅ Using device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")

model = YOLO("yolov8n.pt")
img = numpy.zeros((640, 480, 3), dtype=numpy.uint8)
out = model(img)
print("✅ Inference worked")
