import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Load YOLOv8 model (use yolov8n.pt for speed, or your preferred model)
model = YOLO("yolov8n.pt")  # Download from Ultralytics if needed

try:
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())

        # Run YOLOv8, filter for 'person' class only (class 0)
        results = model(color_image, classes=[0])

        # Draw detections
        annotated_frame = results[0].plot()

        # Display
        cv2.imshow('YOLOv8 Person Detection', annotated_frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
