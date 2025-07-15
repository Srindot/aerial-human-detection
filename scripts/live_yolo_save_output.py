#!/usr/bin/env python3

# Performs yolo on live camera feed and then saves the mp4 file to this_project/yolo_output/*.mp4

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from datetime import datetime

class YOLOv8Recorder:
    def __init__(self):
        rospy.init_node("realsense_yolov8_recorder", anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # Path to your YOLOv8 model

        # Set up output directory and filename
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "../yolo_output")
        os.makedirs(output_dir, exist_ok=True)

        # Filename format: yolo_output_yy_mm_dd_hh_mm_ss.mp4
        timestamp = datetime.now().strftime("%y_%m_%d_%H_%M_%S")
        self.filename = os.path.join(output_dir, f"yolo_output_{timestamp}.mp4")

        self.fps = 30
        self.size_initialized = False
        self.out = None

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("YOLOv8 ROS recorder node initialized.")

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            results = self.model(frame)[0]
            annotated_frame = results.plot()

            if not self.size_initialized:
                h, w = annotated_frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.out = cv2.VideoWriter(self.filename, fourcc, self.fps, (w, h))
                self.size_initialized = True
                rospy.loginfo(f"Recording to: {self.filename} ({w}x{h} @ {self.fps}fps)")

            if self.out:
                self.out.write(annotated_frame)

        except Exception as e:
            rospy.logwarn(f"[YOLO Callback Error] {e}")

    def spin(self):
        rospy.loginfo("Recording started... press Ctrl+C to stop.")
        try:
            rospy.spin()
        finally:
            if self.out:
                self.out.release()
            rospy.loginfo(f"✅ Saved video to: {self.filename}")

if __name__ == "__main__":
    try:
        recorder = YOLOv8Recorder()
        recorder.spin()
    except rospy.ROSInterruptException:
        pass
