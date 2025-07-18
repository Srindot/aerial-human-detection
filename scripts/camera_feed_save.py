#!/usr/bin/env python3

# Records the raw RGB camera feed to this_project/raw_camera_output/raw_camera_output_{time}.mp4

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class RawCameraRecorder:
    def __init__(self):
        rospy.init_node('raw_camera_to_mp4', anonymous=True)
        self.bridge = CvBridge()
        self.fps = rospy.get_param("~fps", 30.0)
        self.width = rospy.get_param("~width", 640)
        self.height = rospy.get_param("~height", 480)
        self.topic = rospy.get_param("~topic", "/camera/color/image_raw")

        # Set up output directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "../raw_camera_output")
        os.makedirs(output_dir, exist_ok=True)

        # Create unique filename
        timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
        self.filename = os.path.join(output_dir, f"raw_camera_output_{timestamp}.mp4")

        # Set up video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(self.filename, fourcc, self.fps, (self.width, self.height))
        rospy.loginfo(f"Recording raw RGB video from {self.topic} to {self.filename} at {self.fps}fps, {self.width}x{self.height}")

        self.sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=10)
        self.frames_recorded = 0

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))

            self.out.write(frame)
            self.frames_recorded += 1

        except Exception as e:
            rospy.logerr(f"[Frame Error] {e}")

    def spin(self):
        rospy.loginfo("Recording raw camera feed... Press Ctrl+C to stop.")
        try:
            rospy.spin()
        finally:
            self.out.release()
            rospy.loginfo(f"✅ Video saved: {self.filename} ({self.frames_recorded} frames)")

if __name__ == "__main__":
    recorder = RawCameraRecorder()
    recorder.spin()
