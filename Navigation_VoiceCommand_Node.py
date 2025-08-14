import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import threading
from collections import deque
import time
import speech_recognition as sr

mic_list = sr.Microphone.list_microphone_names()
print("Available microphones:", mic_list) # To Check if the Mike is working

# To Ensure that the correct mike is used
BUILTIN_MIC_INDEX = 0

# The points in the different rooms in the map.
locations = {
    "beginning": [3.8672661781311035, -1.938331961631775, 0.00351333618164062, 0.000, 0.000, -0.009, 1.000],
    "livingroom": [5.401646614074707, 0.5679720640182495, 0.000102996826171875, 0.0, 0.0, 0.0, 1.0],
    "enterfirst": [7.1726484298706055, 2.5578365325927734, 0.0, 0.0, 0.0, 0.0, 1.0],
    "bedroomone": [4.758491516113281, 5.045022964477539, 0.0, 0.0, 0.0, 0.0, 1.0],
    "entersecond": [-2.736375570297241, 3.4312853813171387, 0.0, 0.0, 0.0, 0.0, 1.0],
    "bedroomtwo": [-2.250734329223633, 6.094691753387451, 0.0, 0.0, 0.0, 0.0, 1.0],
    "kitchenone": [-0.07590034604072571, 7.628844261169434, 0.0, 0.0, 0.0, 0.0, 1.0],
    "kitchentwo": [0.4844783544540405, 9.523351669311523, 0.0, 0.0, 0.0, 0.0, 1.0],
}

class Robot_Assistant(Node):
    def __init__(self):
        super().__init__('robot_assistant')
        self.get_logger().info("Starting Robot Assistant Node")

        
        self.reminder_publisher_ = self.create_publisher(String, '/reminder', 10)
        self.correct_room_publisher_ = self.create_publisher(String, '/correct_room', 10)

        self.correct_room_subscriber = self.create_subscription(
            String, '/correct_room', self.correct_room_callback, 10)

        # Navigation
        self.navigator = BasicNavigator()
        self.set_initial_pose(locations["beginning"])
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Navigation stack ready")

    def command_given(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        self.get_logger().info("Command is given by elderly, Please say the room name")

        while rclpy.ok():
            try:
                with mic as source:
                    recognizer.adjust_for_ambient_noise(source)
                    self.get_logger().info("Listening...")
                    audio = recognizer.listen(source, timeout=5, phrase_time_limit=4)

                command = recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Voice command: {command}")
                
                if "stop" in command:
                    self.get_logger().info("Stop command is given, Stop now!")
                    self.navigator.cancelTask()
                    continue
                for key in locations.keys():
                    if key in command.replace(" ", ""):  
                        msg = String()
                        msg.data = key
                        self.correct_room_publisher_.publish(msg)
                        break

            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                self.get_logger().warn("Invalid Audio")
            except Exception as e:
                self.get_logger().error(f"Error detected: {e}")

    def correct_room_callback(self, msg):
        room_name = msg.data.strip()
        if room_name in locations:
            self.get_logger().info(f"Received coorect room: {room_name}")
            self.navigate_to_target(locations[room_name])
        else:
            self.get_logger().warn(f"Unknown room: {room_name}")

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

    def navigate_to_target(self, pose, timeout=240.0):
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

    def wait_until_navigation_complete(self, timeout=240.0):
        start_time = time.time()
        while not self.navigator.isTaskComplete():
            elapsed_time = time.time() - start_time
            self.get_logger().info(f"Navigation elapsed time: {elapsed_time:.1f} seconds")

            if elapsed_time > timeout:
                self.get_logger().warn(f"Navigation timeout after {timeout} seconds. The Task is being cancelled.")
                self.navigator.cancelTask()
                break

            time.sleep(0.5)

        result = self.navigator.getResult()
        return result == TaskResult.SUCCEEDED

def main(args=None):
    rclpy.init(args=args)
    robbot_assistant = Robot_Assistant()
    executor = MultiThreadedExecutor()
    executor.add_node(robbot_assistant)

    try:
        executor.spin()
    finally:
        robbot_assistant.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

