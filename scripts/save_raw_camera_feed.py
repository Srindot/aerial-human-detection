# saves the raw rgb camera feed from the realsense camera

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
out = cv2.VideoWriter('rgb_output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30.0, (640, 480))

def callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    out.write(frame)

rospy.init_node('rgb_recorder')
rospy.Subscriber('/camera/color/image_raw', Image, callback)
rospy.spin()
out.release()
