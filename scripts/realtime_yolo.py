#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloV8LiveDetector:
    def __init__(self):
        rospy.init_node("yolov8_live_detector", anonymous=True)

        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.model_path = rospy.get_param("~model_path", "yolov8n.pt")  # update if needed

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)  # Automatically downloads if not available
        rospy.loginfo(f"Loaded YOLOv8 model from: {self.model_path}")

        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLOv8 inference
            results = self.model(cv_image)

            # Annotate frame
            annotated_frame = results[0].plot()

            # Show image
            cv2.imshow("YOLOv8 Detection", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Image callback error: {e}")

    def run(self):
        rospy.loginfo("YOLOv8 RealSense detector node started")
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = YoloV8LiveDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
