yolo follower:
tb:
ros2 launch camera_ros camera.launch.py
ros2 launch tb3_launcher turtlebot3.launch.py slam:=True
sutd:
ros2 run yolo_follower yolo_follower_node


voice command:
ros2 run voice_command voice_command_node

r a
tb:
ros2 launch tb3_launcher turtlebot3.launch.py map:=${HOME}/tb_ws/src/tb3_launcher/maps/dyson_studio_map3.yaml

sutd
rviz2
ros2 launch robot_assistant robot_assistant.py
ros2 topic pub /target_room std_msgs/String "data: 'room1'"
