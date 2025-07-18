# yolo_detector.py

from ultralytics import YOLO
import cv2

class YOLODetector:
    def __init__(self, model_path="yolov8n.pt", target_class="person"):
        self.model = YOLO(model_path)
        self.target_class = target_class
        self.names = self.model.names

    def detect(self, frame, draw=True):
        results = self.model(frame)[0]

        if not draw:
            return frame, results

        # Filter and draw only target_class
        filtered_frame = frame.copy()
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = self.names[cls_id]

            if class_name == self.target_class:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = f"{class_name} {conf:.2f}"
                cv2.rectangle(filtered_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(filtered_frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return filtered_frame, results


    def draw_boxes(self, frame, results):
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if self.names[cls_id] == self.target_class:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = f"{self.names[cls_id]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame
