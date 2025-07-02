#!/usr/bin/env python3

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import glob
import os

# -------------------------------
# Config
BAG_DIR = "src/realsense_human_detector/"
RGB_TOPIC = "/camera/color/image_raw"
MODEL_PATH = "yolov8n.pt"  # or yolov8s.pt
# -------------------------------

def find_latest_bag(bag_dir):
    bags = sorted(glob.glob(os.path.join(bag_dir, "*.bag")), key=os.path.getmtime)
    return bags[-1] if bags else None

def main():
    rospy.init_node("rosbag_yolo_detector", anonymous=True)
    bridge = CvBridge()

    bag_path = find_latest_bag(BAG_DIR)
    if not bag_path:
        rospy.logerr("No .bag file found in {}".format(BAG_DIR))
        return

    print(f"[INFO] Loading YOLOv8 model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

    print(f"[INFO] Opening bag: {bag_path}")
    bag = rosbag.Bag(bag_path, 'r')

    for topic, msg, t in bag.read_messages(topics=[RGB_TOPIC]):
        try:
            # Convert ROS Image to OpenCV format
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLOv8 detection
            results = model(cv_img)[0]

            # Plot results on image
            annotated_frame = results.plot()

            # Show the annotated frame
