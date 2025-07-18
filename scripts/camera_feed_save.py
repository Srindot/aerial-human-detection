#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import time

class RawCameraRecorder:
    def __init__(self):
        rospy.init_node('raw_camera_to_mp4', anonymous=True)
        self.bridge = CvBridge()

        self.topic = rospy.get_param("~topic", "/camera/color/image_raw")
        self.measure_frames = rospy.get_param("~measure_frames", 10)  # frames to measure FPS

        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "../raw_camera_output")
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
        self.filename = os.path.join(output_dir, f"raw_camera_output_{timestamp}.mp4")

        self.out = None
        self.frames_recorded = 0

        self.frame_times = []
        self.start_time = None

        self.sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=10)
        rospy.loginfo(f"🎥 Subscribed to {self.topic} — waiting for image data...")

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            curr_time = time.time()
            if self.start_time is None:
                self.start_time = curr_time

            # Collect frame timestamps until measure_frames reached
            if len(self.frame_times) < self.measure_frames:
                self.frame_times.append(curr_time)
                # Just keep appending but don't record video yet
                return

            # Once we have enough timestamps, calculate FPS and init video writer
            if self.out is None:
                intervals = [t2 - t1 for t1, t2 in zip(self.frame_times[:-1], self.frame_times[1:])]
                avg_interval = sum(intervals) / len(intervals)
                measured_fps = 1.0 / avg_interval if avg_interval > 0 else 30.0

                height, width = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.out = cv2.VideoWriter(self.filename, fourcc, measured_fps, (width, height))
                rospy.loginfo(f"📹 Recording started: {self.filename} ({width}x{height} @ {measured_fps:.2f} FPS)")

            # Write frames to video
            self.out.write(frame)
            self.frames_recorded += 1

        except Exception as e:
            rospy.logerr(f"[Frame Error] {e}")

    def spin(self):
        rospy.loginfo("🔴 Recording raw camera feed... Press Ctrl+C to stop.")
        try:
            rospy.spin()
        finally:
            if self.out:
                self.out.release()
                duration = time.time() - self.start_time if self.start_time else 0
                avg_fps = self.frames_recorded / duration if duration > 0 else 0
                rospy.loginfo(f"✅ Video saved: {self.filename} ({self.frames_recorded} frames, avg FPS: {avg_fps:.2f})")

if __name__ == "__main__":
    recorder = RawCameraRecorder()
    recorder.spin()
