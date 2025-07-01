#!/usr/bin/env python3

import rospy
import zmq
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageZMQPublisher:
    def __init__(self):
        rospy.init_node('image_zmq_publisher', anonymous=True)

        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.zmq_pub_port = rospy.get_param('~zmq_pub_port', '5555')  # Or pass full "tcp://*:5555"

        self.bridge = CvBridge()

        # Setup ZMQ publisher
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.bind(f"tcp://*:{self.zmq_pub_port}")

        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to {self.image_topic} and publishing over ZMQ on port {self.zmq_pub_port}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Encode as JPEG for sending
            _, encoded = cv2.imencode('.jpg', cv_image)
            self.publisher.send(encoded.tobytes())

        except Exception as e:
            rospy.logerr(f"ZMQ Publish Error: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ImageZMQPublisher()
    node.run()
