#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
from yolo_detector import YOLODetector

class YOLOBoundingBoxPublisher:
    def __init__(self):
        rospy.init_node('yolo_boundingbox_publisher', anonymous=True)

        self.bridge = CvBridge()
        self.model_path = rospy.get_param("~model_path", "yolov8n.pt")
        self.topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.target_class = rospy.get_param("~target_class", "person")

        self.detector = YOLODetector(model_path=self.model_path, target_class=self.target_class)

        self.pub = rospy.Publisher("/yolo_bounding_boxes", String, queue_size=10)
        self.sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=1)

        rospy.loginfo(f"YOLO Bounding Box Publisher started. Subscribed to {self.topic}")

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, results = self.detector.detect(frame, draw=False)

            bbox_list = []
            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.detector.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                bbox_list.append({
                    "class": class_name,
                    "confidence": round(conf, 3),
                    "bbox": [x1, y1, x2, y2]
                })

            # Publish bounding boxes as a JSON string
            self.pub.publish(json.dumps(bbox_list))

        except Exception as e:
            rospy.logerr(f"[YOLO BBox Callback Error] {e}")

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = YOLOBoundingBoxPublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass
