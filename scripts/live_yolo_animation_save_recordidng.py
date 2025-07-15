#!/usr/bin/env python3

# Performs yolo on live camera feed and then saves the output dir 

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

# Change these import paths as needed for your custom message
from your_msgs.msg import BoundingBoxes, BoundingBox   # <-- Define these or install a package that provides them

class YOLOv8Detector:
    def __init__(self):
        rospy.init_node("realsense_yolov8_detector", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        self.bbox_pub = rospy.Publisher("yolov8_bounding_boxes", BoundingBoxes, queue_size=1)
        self.model = YOLO("yolov8n.pt")  # or 'yolov8s.pt', path to your model

    def callback(self, data):
        try:
            # Convert ROS Image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            results = self.model(cv_img)[0]

            # Prepare bounding boxes message
            bbox_msg = BoundingBoxes()
            bbox_msg.header = data.header

            for box in results.boxes:
                # YOLO returns: xyxy (left, top, right, bottom)
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cls_id = int(box.cls[0]) if hasattr(box, 'cls') else -1
                score = float(box.conf[0]) if hasattr(box, 'conf') else 0.0

                bbox = BoundingBox()
                bbox.xmin = x1
                bbox.ymin = y1
                bbox.xmax = x2
                bbox.ymax = y2
                bbox.class_id = cls_id
                bbox.probability = score
                bbox.class_name = self.model.names[cls_id] if cls_id >= 0 else ""

                bbox_msg.bounding_boxes.append(bbox)

                # Draw for visualization
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_img, bbox.class_name, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

            # Publish bounding boxes
            self.bbox_pub.publish(bbox_msg)

            # Optional: show the image (remove for full headless use)
            cv2.imshow("YOLOv8 Detection", cv_img)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn("YOLOv8 callback error: %s", e)

if __name__ == "__main__":
    try:
        YOLOv8Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
