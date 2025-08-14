import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from playsound import playsound
 
from ament_index_python.packages import get_package_share_directory
from collections import deque
from threading import Thread
import os
import time
from datetime import datetime
 
class ReminderCheckin(Node):
 
    def __init__(self):
        super().__init__('reminder_checkin')
        
        # Create reminder Queue & reminder History
        self.reminder_queue = deque()
        self.reminder_history = {}

        # Get package installation directory
        self.pkg_dir = get_package_share_directory("reminder_checkin")
        # Audio directory (Within installation directory)
        self.audio_dir = os.path.join(self.pkg_dir, 'audio')
        
        # Create Subscriber
        self.subscriber = self.create_subscription(String, '/reminder', self.reminder_callback, 10)
        # Create playback thread
        self.playback_thread = Thread(target=self.playback, daemon=True)
        self.playback_thread.start()
        
        # Timer for scheduled reminders every 10 seconds
        self.create_timer(10.0, self.reminder_timer_callback)
        
        self.previous_trigger = None
        
        self.get_logger().info("Reminder Checkin Node Started!")
    
    def __del__(self):
        print (self.reminder_history)

    def playback(self):
        while rclpy.ok():
            if len(self.reminder_queue) > 0:
                cmd = self.reminder_queue.pop()
                reminder_file = os.path.join(self.audio_dir, cmd + '.mp3')
                if os.path.exists(reminder_file):
                    # Record History
                    if cmd in self.reminder_history:
                        self.reminder_history[cmd] += 1
                    else:
                        self.reminder_history[cmd] = 1
                    # Show history
                    self.get_logger().info(cmd + " : Total Count : " + str(self.reminder_history[cmd]))
                else:
                    self.get_logger().warn("Reminder Checkin Error. Received : " + cmd)
                    reminder_file = os.path.join(self.audio_dir, 'incorrect value.mp3')
                
                # Play Reminder
                playsound(reminder_file)

            # Loop rate limiter (10Hz)
            time.sleep(0.1)

    def reminder_callback(self, msg):
        reminder_phrase = msg.data.lower()
        self.reminder_queue.append(reminder_phrase)
        
        
    def reminder_timer_callback(self):
        present_time = datetime.now().strftime('%H:%M')
        self.get_logger().info("The Timer is triggered at {present_time}")
        reminder_timings = {
            '08:00': 'morning_checkin',
            '10:05': 'morning_medication',
            '12:00': 'lunch_reminder',
            '15:00': 'friend_reminder',
            '16:00': 'teatime_checkin',
            '20:00': 'night_medication'
        }
        

        if present_time in reminder_timings:
           if self.previous_trigger != present_time:  
             self.get_logger().info("Reminder activated: {reminder_timings[present_time]}")
             self.reminder_queue.append(reminder_timings[present_time])
             self.previous_trigger = present_time  
           else:
             self.get_logger().debug("Reminder has already been triggered at {present_time}")
 
def main(args=None):
    rclpy.init(args=args)
 
    reminder_checkin = ReminderCheckin()
 
    rclpy.spin(reminder_checkin)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
