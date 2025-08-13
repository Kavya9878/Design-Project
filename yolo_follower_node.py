#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO
import numpy as np

class YoloFollower(Node):
    def __init__(self):
        super().__init__('yolo_follower')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.model = YOLO('yolov8n.pt')  # load model
        self.declare_parameter('image_width', 640)
        self.image_width = self.get_parameter('image_width').value

    def image_callback(self, msg):
        self.get_logger().info("Image received!")
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(frame, verbose=False)[0]

        persons = [r for r in results.boxes if int(r.cls[0]) == 0]  # class 0 is person
        if persons:
            largest = max(persons, key=lambda r: r.xywh[0][2])  # pick widest person box
            x_center = int(largest.xywh[0][0].item())

            error = (self.image_width // 2) - x_center

            twist = Twist()
            twist.linear.x = 0.12
            twist.angular.z = float(error) / 300.0  # tune as needed
            self.get_logger().info(f"error: {error}, angular.z: {twist.angular.z}")
            self.cmd_pub.publish(twist)
        else:
            self.cmd_pub.publish(Twist())  # stop if no person
        
        for r in results.boxes:
            cls_id = int(r.cls[0])
            # 只显示“人”类别
            if cls_id == 0:
                # xyxy格式坐标，转换成整数
                xyxy = r.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy

                # 在图像上画矩形框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 写上标签
                cv2.putText(frame, 'Person', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # 显示图像窗口
        cv2.imshow('YOLO Person Detection', frame)
        cv2.waitKey(1)  # 1ms，必要以刷新窗口

def main(args=None):
    rclpy.init(args=args)
    node = YoloFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
