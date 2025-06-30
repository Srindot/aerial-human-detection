#!/usr/bin/env python3

import rospy
import cv2
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YoloDetector:
    def __init__(self):
        rospy.init_node("yolo_detector_node")

        # Parameters
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.bbox_topic = rospy.get_param("~bbox_topic", "/yolo/bboxes")
        self.model_path = rospy.get_param("~model_path", "/home/container_user/models/yolov8n.pt")

        # Init tools
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.bbox_topic, Detection2DArray, queue_size=10)

        # Load model BEFORE subscribing
        try:
            rospy.loginfo(f"Loading YOLO model from: {self.model_path}")
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path)
            self.model.conf = 0.4
            rospy.loginfo("YOLO model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            rospy.signal_shutdown("Model load failed")
            return

        # Now that the model is loaded, subscribe
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def image_callback(self, msg):
        if not hasattr(self, 'model'):
            rospy.logwarn("Model not loaded yet.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)

            detections = Detection2DArray()
            detections.header = msg.header

            for *xyxy, conf, cls in results.xyxy[0]:
                if int(cls) == 0:  # Class 0 = person
                    det = Detection2D()
                    det.header = msg.header

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = int(cls)
                    hypothesis.score = float(conf)
                    det.results.append(hypothesis)

                    x1, y1, x2, y2 = map(float, xyxy)
                    det.bbox.center.position.x = (x1 + x2) / 2.0
                    det.bbox.center.position.y = (y1 + y2) / 2.0
                    det.bbox.size_x = x2 - x1
                    det.bbox.size_y = y2 - y1
                    detections.detections.append(det)

            self.pub.publish(detections)

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = YoloDetector()
    node.run()
