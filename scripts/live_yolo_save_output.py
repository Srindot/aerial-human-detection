#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from yolo_detector import YOLODetector
import time

class YOLOVideoRecorder:
    def __init__(self):
        rospy.init_node("realsense_yolov8_recorder", anonymous=True)
        self.bridge = CvBridge()

        # Parameters
        self.model_path = rospy.get_param("~model_path", "yolov8n.pt")
        self.topic = rospy.get_param("~topic", "/camera/color/image_raw")
        self.target_class = rospy.get_param("~target_class", "person")
        self.fps_param = rospy.get_param("~fps", 30)  # still keep param for reference

        self.detector = YOLODetector(model_path=self.model_path, target_class=self.target_class)

        # Output video path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "../yolo_output")
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%y_%m_%d_%H_%M_%S")
        self.filename = os.path.join(output_dir, f"yolo_output_{timestamp}.mp4")

        self.out = None
        self.size_initialized = False

        # For FPS calculation
        self.prev_time = None
        self.curr_fps = 0

        self.sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(f"YOLOv8 Recorder Node initialized. Subscribed to {self.topic}")

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Calculate FPS
            curr_time = time.time()
            if self.prev_time is not None:
                elapsed = curr_time - self.prev_time
                if elapsed > 0:
                    self.curr_fps = 1.0 / elapsed
            self.prev_time = curr_time

            annotated_frame, _ = self.detector.detect(frame, draw=True)

            # Put FPS text on bottom-left corner
            fps_text = f"FPS: {self.curr_fps:.1f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_thickness = 2
            text_size, _ = cv2.getTextSize(fps_text, font, font_scale, font_thickness)
            text_x = 10
            text_y = annotated_frame.shape[0] - 10  # 10 px from bottom

            # Draw black rectangle background for readability
            cv2.rectangle(
                annotated_frame,
                (text_x - 5, text_y - text_size[1] - 5),
                (text_x + text_size[0] + 5, text_y + 5),
                (0, 0, 0),
                thickness=cv2.FILLED,
                lineType=cv2.LINE_AA,
            )
            # Put white text on top
            cv2.putText(
                annotated_frame,
                fps_text,
                (text_x, text_y),
                font,
                font_scale,
                (255, 255, 255),
                font_thickness,
                lineType=cv2.LINE_AA,
            )

            # Initialize video writer once actual FPS is known
            if not self.size_initialized and self.curr_fps > 1.0:
                h, w = annotated_frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.out = cv2.VideoWriter(self.filename, fourcc, self.curr_fps, (w, h))
                self.size_initialized = True
                rospy.loginfo(f"Recording to {self.filename} ({w}x{h} @ {self.curr_fps:.1f} fps)")

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
