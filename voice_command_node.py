import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import subprocess
import os

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher_ = self.create_publisher(String, 'voice_text', 10)
        self.timer = self.create_timer(5.0, self.listen_and_publish)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        self.yolo_process = None

    def listen_and_publish(self):
        with self.microphone as source:
            self.get_logger().info("Listening...")
            audio = self.recognizer.listen(source)

        try:
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"Recognized: {text}")
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            
            # Check for command
            if "follow me" in text:
                if self.yolo_process is None:
                    self.get_logger().info("🚀 Launching YOLO follower node...")
                    self.yolo_process = subprocess.Popen(
                        ["ros2", "run", "yolo_follower", "yolo_follower_node"],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
    )
                else:
                    self.get_logger().info("ℹ️ YOLO follower node already running.")
            elif "stop" in text or "stop following" in text:
                if self.yolo_process:
                    self.get_logger().info("🛑 Stopping YOLO follower node...")
                    self.yolo_process.terminate()
                    self.yolo_process = None
                else:
                    self.get_logger().info("ℹ️ YOLO follower node not running.")

        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"API error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

