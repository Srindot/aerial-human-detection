#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from yolo_detector import YOLODetector # Assuming you have this file
import time

class YOLOVideoRecorder:
    def __init__(self):
        rospy.init_node("realsense_yolov8_recorder", anonymous=True)
        self.bridge = CvBridge()

        # Parameters
        self.model_path = rospy.get_param("~model_path", "yolov8n.pt")
        self.topic = rospy.get_param("~topic", "/camera/color/image_raw")
        self.target_class = rospy.get_param("~target_class", "person")
        # New parameter for stable FPS calculation
        self.measure_frames = rospy.get_param("~measure_frames", 30)

        self.detector = YOLODetector(model_path=self.model_path, target_class=self.target_class)

        # Output video path setup
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "../yolo_output")
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%y_%m_%d_%H_%M_%S")
        self.filename = os.path.join(output_dir, f"yolo_output_{timestamp}.mp4")

        # Video writer and FPS calculation attributes
        self.out = None
        self.frame_times = []
        self.start_time = None
        self.frames_recorded = 0

        self.sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo(f"YOLOv8 Recorder Node initialized. Subscribed to {self.topic}")
        rospy.loginfo(f"Measuring FPS over the first {self.measure_frames} frames...")

    def callback(self, data):
        try:
            curr_time = time.time()
            if self.start_time is None:
                self.start_time = curr_time

            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            annotated_frame, _ = self.detector.detect(frame, draw=True)

            # --- FPS Calculation & VideoWriter Initialization (New Logic) ---
            if self.out is None:
                # 1. Collect timestamps for the first `measure_frames`
                if len(self.frame_times) < self.measure_frames:
                    self.frame_times.append(curr_time)
                    # Display status on frame while measuring
                    cv2.putText(annotated_frame, f"Measuring FPS... {len(self.frame_times)}/{self.measure_frames}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                # 2. Once enough frames are measured, calculate average FPS and init writer
                else:
                    intervals = [t2 - t1 for t1, t2 in zip(self.frame_times[:-1], self.frame_times[1:])]
                    avg_interval = sum(intervals) / len(intervals)
                    measured_fps = 1.0 / avg_interval if avg_interval > 0 else 30.0

                    h, w = annotated_frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    self.out = cv2.VideoWriter(self.filename, fourcc, measured_fps, (w, h))
                    rospy.loginfo(f"📹 Recording started: {self.filename} ({w}x{h} @ {measured_fps:.2f} FPS)")

            # --- Write Frame and Display Running FPS ---
            if self.out:
                self.frames_recorded += 1
                # Calculate a running average FPS for display
                elapsed_time = time.time() - self.start_time
                running_avg_fps = self.frames_recorded / elapsed_time if elapsed_time > 0 else 0

                # Put running FPS text on the frame
                fps_text = f"FPS: {running_avg_fps:.1f}"
                cv2.putText(annotated_frame, fps_text, (10, annotated_frame.shape[0] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(annotated_frame, fps_text, (10, annotated_frame.shape[0] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                self.out.write(annotated_frame)
            
            # You might want to display the frame for real-time feedback (optional)
            # cv2.imshow("YOLOv8 Detections", annotated_frame)
            # cv2.waitKey(1)

        except Exception as e:
            rospy.logwarn(f"[YOLO Callback Error] {e}")

    def spin(self):
        rospy.loginfo("Recording node is running... Press Ctrl+C to stop.")
        try:
            rospy.spin()
        finally:
            if self.out:
                self.out.release()
                # cv2.destroyAllWindows() # Uncomment if using cv2.imshow
                
                # Calculate final average FPS for the saved video
                duration = time.time() - self.start_time if self.start_time else 0
                final_avg_fps = self.frames_recorded / duration if duration > 0 else 0
                rospy.loginfo(f"✅ Video saved: {self.filename}")
                rospy.loginfo(f"   - Total Frames: {self.frames_recorded}")
                rospy.loginfo(f"   - Duration: {duration:.2f} seconds")
                rospy.loginfo(f"   - Average FPS: {final_avg_fps:.2f}")


if __name__ == "__main__":
    try:
        recorder = YOLOVideoRecorder()
        recorder.spin()
    except rospy.ROSInterruptException:
        pass