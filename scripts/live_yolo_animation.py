#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLORealsenseNode:
    def __init__(self):
        rospy.init_node('yolo_realsense_detector', anonymous=True)

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # or yolov8s.pt, yolov8m.pt, etc.

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        rospy.loginfo("YOLOv8 Realsense Detector Node Started")
    
    def callback(self, data):
        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Could not convert image: %s", e)
            return

        # Run YOLOv8 inference
        results = self.model(frame)[0]

        # Draw bounding boxes for people
        for box in results.boxes:
            cls = int(box.cls[0])
            if self.model.names[cls] == 'person':
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                label = f"{self.model.names[cls]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the frame
        cv2.imshow("YOLOv8 Realsense People Detection", frame)
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
