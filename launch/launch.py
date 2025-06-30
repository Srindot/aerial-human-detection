from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_realsense_detector',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[{
                'camera_topic': '/camera/color/image_raw',
                'bbox_topic': '/yolo/bboxes',
                'model_path': 'models/yolov8n.pt'
            }]
        )
    ])
