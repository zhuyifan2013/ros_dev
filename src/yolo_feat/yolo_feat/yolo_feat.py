import rclpy
from rclpy.node import Node
import cv2
import numpy as np
# import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_feat')

        # 订阅摄像头话题（这里用的是 usb_cam 默认的话题）
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )
        self.subscription  # 避免未使用的变量警告

        # 发布检测结果
        self.publisher_ = self.create_publisher(String, '/yolo_detections', 10)

        # 加载 YOLO 模型
        self.model = YOLO('yolo11n.pt')  # 这里换成你的模型
        self.bridge = CvBridge()

        self.get_logger().info("YOLO Detector Node Initialized.")

    def image_callback(self, msg):
        # 将 ROS Image 转换为 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 运行 YOLO 进行目标检测
        results = self.model(cv_image)

        detections = []
        for result in results:
            for box in result.boxes:
                label = result.names[int(box.cls.item())]  # 目标类别
                confidence = box.conf.item()  # 置信度
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  # 边界框坐标

                detections.append(f"{label} ({confidence:.2f}) at [{x1}, {y1}, {x2}, {y2}]")

        # 发布检测结果
        detection_msg = String()
        detection_msg.data = "; ".join(detections)
        self.publisher_.publish(detection_msg)

        self.get_logger().info(f"Published detections: {detection_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
