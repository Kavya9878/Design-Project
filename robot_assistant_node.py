import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from yolo_detector_msgs.msg import BoundingBox
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import threading
from collections import deque
import time

# 位置定义
locations = {
    "start": [3.8672661781311035, -1.938331961631775, 0.00351333618164062, 0.000, 0.000, -0.009, 1.000],
    "room1": [5.401646614074707, 0.5679720640182495, 0.000102996826171875, 0.0, 0.0, 0.0, 1.0],
    "room2": [7.1726484298706055, 2.5578365325927734, 0.0, 0.0, 0.0, 0.0, 1.0],
    "room21": [4.758491516113281, 5.045022964477539, 0.0, 0.0, 0.0, 0.0, 1.0],
    "room3": [-2.736375570297241, 3.4312853813171387, 0.0, 0.0, 0.0, 0.0, 1.0],
    "room31": [-2.250734329223633, 6.094691753387451, 0.0, 0.0, 0.0, 0.0, 1.0],
    "room4": [-0.07590034604072571, 7.628844261169434, 0.0, 0.0, 0.0, 0.0, 1.0],
    "room41": [0.4844783544540405, 9.523351669311523, 0.0, 0.0, 0.0, 0.0, 1.0],
}
class RI_Challenge(Node):
    def __init__(self):
        super().__init__('ri_challenge')
        self.get_logger().info("Starting RI Challenge Node")

        # 发布器
        self.alert_publisher_ = self.create_publisher(String, '/alert', 10)

        # 订阅器
        self.bounding_box_subscriber = self.create_subscription(
            BoundingBox, '/bounding_boxes', self.bounding_box_callback, 10)

        self.detection_image_subscriber = self.create_subscription(
            CompressedImage, '/image_output_topic/compressed', self.detection_output_image_callback, 10)

        self.target_room_subscriber = self.create_subscription(
            String, '/target_room', self.target_room_callback, 10)

        # 导航控制
        self.navigator = BasicNavigator()
        self.set_initial_pose(locations["start"])
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Navigation stack ready")

        # 检测参数
        self.detection_queue = deque()
        self.detection_frame_count = 0
        self.detection_label = ""

    def target_room_callback(self, msg):
        room_name = msg.data.strip()
        if room_name in locations:
            self.get_logger().info(f"Received target room: {room_name}")
            self.navigate_to_target(locations[room_name])

            # 检测是否有人
            intruder_detected = self.detect('person', frame_count=4, min_detections=2)
            if intruder_detected:
                self.send_alert(f"intruder at {room_name}")
            else:
                self.send_alert(f"clear at {room_name}")
        else:
            self.get_logger().warn(f"Unknown room: {room_name}")

    def detect(self, label, frame_count=1, min_detections=1, timeout=10.0):
        self.detection_label = label
        self.detection_queue.clear()
        self.detection_frame_count = 0
        self.detection_start_time = time.time()

        while self.detection_frame_count < frame_count:
            if timeout < (time.time() - self.detection_start_time):
                self.get_logger().warn(f"Detection Timeout: {label}")
                return False
            time.sleep(0.1)

        if len(self.detection_queue) >= min_detections:
            self.get_logger().info(f"Detected: {label}")
            return True
        else:
            return False

    def detection_output_image_callback(self, msg):
        self.detection_frame_count += 1

    def bounding_box_callback(self, msg):
        if self.detection_label != msg.label:
            return
        self.detection_queue.append(msg)

    def set_initial_pose(self, pose):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = pose[0]
        initial_pose.pose.position.y = pose[1]
        initial_pose.pose.position.z = pose[2]
        initial_pose.pose.orientation.x = pose[3]
        initial_pose.pose.orientation.y = pose[4]
        initial_pose.pose.orientation.z = pose[5]
        initial_pose.pose.orientation.w = pose[6]

        self.get_logger().info("Setting initial pose")
        self.navigator.setInitialPose(initial_pose)

    def navigate_to_target(self, pose, timeout=60.0):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = pose[2]
        goal_pose.pose.orientation.x = pose[3]
        goal_pose.pose.orientation.y = pose[4]
        goal_pose.pose.orientation.z = pose[5]
        goal_pose.pose.orientation.w = pose[6]

        self.get_logger().info(f"Navigating to: {pose}")
        self.navigator.goToPose(goal_pose)
        self.wait_until_navigation_complete(timeout=timeout)

    def wait_until_navigation_complete(self, timeout=60.0):
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=timeout):
                self.navigator.cancelTask()
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation done")
            return True
        else:
            self.get_logger().error("Navigation failed")
            return False

    def send_alert(self, msg):
        alert_msg = String()
        alert_msg.data = msg
        self.alert_publisher_.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    ri_challenge = RI_Challenge()
    executor = MultiThreadedExecutor()
    executor.add_node(ri_challenge)

    try:
        executor.spin()
    finally:
        ri_challenge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

