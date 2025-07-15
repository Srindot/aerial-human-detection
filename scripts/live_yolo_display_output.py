#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from yolo_detector import YOLODetector  # <-- Your modular class

class YOLORealsenseNode:
    def __init__(self):
        rospy.init_node('yolo_realsense_detector', anonymous=True)

        # Parameters (optional)
        self.model_path = rospy.get_param("~model_path", "yolov8n.pt")
        self.topic = rospy.get_param("~topic", "/camera/color/image_raw")
        self.target_class = rospy.get_param("~target_class", "person")

        self.bridge = CvBridge()
        self.detector = YOLODetector(model_path=self.model_path, target_class=self.target_class)

        self.image_sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=1)
        rospy.loginfo(f"YOLOv8 Realsense Detector Node Started. Subscribed to {self.topic}")

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")
            return

        # Inference and annotation
        annotated_frame, _ = self.detector.detect(frame, draw=True)

        # Display result
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        node = YOLORealsenseNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
