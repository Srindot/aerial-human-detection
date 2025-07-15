#!/usr/bin/env python3

# Records YOLO-annotated camera feed to this_project/yolo_output/yolo_output_yy_mm_dd_hh_mm_ss.mp4

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from yolo_detector import YOLODetector  # <-- Modular class

class YOLOVideoRecorder:
    def __init__(self):
        rospy.init_node("realsense_yolov8_recorder", anonymous=True)
        self.bridge = CvBridge()

        # Parameters
        self.model_path = rospy.get_param("~model_path", "yolov8n.pt")
        self.topic = rospy.get_param("~topic", "/camera/color/image_raw")
        self.target_class = rospy.get_param("~target_class", "person")
        self.fps = rospy.get_param("~fps", 30)

        # Initialize YOLO detector
        self.detector = YOLODetector(model_path=self.model_path, target_class=self.target_class)

        # Setup output video path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "../yolo_output")
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%y_%m_%d_%H_%M_%S")
        self.filename = os.path.join(output_dir, f"yolo_output_{timestamp}.mp4")

        self.out = None
        self.size_initialized = False

        self.sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(f"YOLOv8 Recorder Node initialized. Subscribed to {self.topic}")

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Run detection and get annotated frame
            annotated_frame, _ = self.detector.detect(frame, draw=True)

            # Initialize video writer if needed
            if not self.size_initialized:
                h, w = annotated_frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.out = cv2.VideoWriter(self.filename, fourcc, self.fps, (w, h))
                self.size_initialized = True
                rospy.loginfo(f"Recording to {self.filename} ({w}x{h} @ {self.fps} fps)")

            if self.out:
                self.out.write(annotated_frame)

        except Exception as e:
            rospy.logwarn(f"[YOLO Callback Error] {e}")

    def spin(self):
        rospy.loginfo("Recording started... Press Ctrl+C to stop.")
        try:
            rospy.spin()
        finally:
            if self.out:
                self.out.release()
            rospy.loginfo(f"✅ Video saved to: {self.filename}")

if __name__ == "__main__":
    try:
        recorder = YOLOVideoRecorder()
        recorder.spin()
    except rospy.ROSInterruptException:
        pass
