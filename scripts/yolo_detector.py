import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import torch

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('bbox_topic', '/yolo/bboxes')
        self.declare_parameter('model_path', 'models/yolov8n.pt')

        self.camera_topic = self.get_parameter('camera_topic').value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.model_path = self.get_parameter('model_path').value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Detection2DArray, self.bbox_topic, 10)
        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path)
        self.model.conf = 0.4
        self.get_logger().info("YOLO model loaded")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)
            detections = Detection2DArray()

            for *xyxy, conf, cls in results.xyxy[0]:
                if int(cls) == 0:  # class 0 usually means person
                    detection = Detection2D()
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = 0
                    hypothesis.score = float(conf)
                    detection.results.append(hypothesis)
                    x1, y1, x2, y2 = map(int, xyxy)
                    detection.bbox.center.position.x = (x1 + x2) / 2.0
                    detection.bbox.center.position.y = (y1 + y2) / 2.0
                    detection.bbox.size_x = x2 - x1
                    detection.bbox.size_y = y2 - y1
                    detections.detections.append(detection)

            detections.header = msg.header
            self.publisher.publish(detections)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
