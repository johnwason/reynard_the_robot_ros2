#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import reynard_the_robot_ros2_msgs.msg as reynard_msg
import reynard_the_robot_ros2_msgs.srv as reynard_srv
import numpy as np
import time
import threading

rclpy.init()

# Initialize a rospy node
ros_node = Node('reynard_the_robot_client')

# Create a multi-threaded executor
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(ros_node)

# Start the executor
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

# Create service proxies, publishers, and subscribers
teleport_srv = ros_node.create_client(reynard_srv.Teleport, '/reynard/teleport')
say_pub = ros_node.create_publisher(String, '/reynard/say', 10)
new_message_sub = ros_node.create_subscription(String, '/reynard/new_message', lambda data: print(f"New message: {data.data}"), 10)
drive_robot_srv = ros_node.create_client(reynard_srv.Drive, '/reynard/drive_robot')
drive_arm_srv = ros_node.create_client(reynard_srv.Drive, '/reynard/drive_arm')
set_arm_position_srv = ros_node.create_client(reynard_srv.SetPosition, '/reynard/set_arm_position')
set_color_srv = ros_node.create_client(reynard_srv.SetColor, '/reynard/set_color')
get_color_srv = ros_node.create_client(reynard_srv.GetColor, '/reynard/get_color')

new_state_print_count = [0]

# Define a callback function for the new state subscriber
def new_state(data):
    if new_state_print_count[0] < 2:
        print(f"New state: {data}")
    new_state_print_count[0] += 1
new_state_pub = ros_node.create_subscription(reynard_msg.ReynardState, '/reynard/state', new_state, 10)

teleport_srv.wait_for_service()

# Teleport the robot
teleport_req = reynard_srv.Teleport.Request(x=0.1, y=-0.2)
teleport_res = teleport_srv.call(teleport_req)
assert teleport_res.success, f"Teleport error: {teleport_res.status_message}"

# Drive the robot with no timeout
drive_res = drive_robot_srv.call(reynard_srv.Drive.Request(velocity=np.array((0.5,-0.2)),timeout=-1.0,wait=False))
assert drive_res.success, f"Drive error: {drive_res.status_message}"

# Wait for one second
time.sleep(1)

# Stop the robot
drive_res = drive_robot_srv.call(reynard_srv.Drive.Request(velocity=np.array((0,0)),timeout=-1.0,wait=False))
assert drive_res.success, f"Drive error: {drive_res.status_message}"

# Set the arm position
pos_res = set_arm_position_srv.call(reynard_srv.SetPosition.Request(target_position=np.deg2rad((100,-30,-70))))
assert pos_res.success, f"Set arm position error: {pos_res.status_message}"

# Drive the arm with no timeout
drive_res = drive_arm_srv.call(reynard_srv.Drive.Request(velocity=np.deg2rad((10, -30, -15)), timeout=1.5, wait=True))
assert drive_res.success, f"Drive error: {drive_res.status_message}"

# Set the color to red
color_res = set_color_srv.call(reynard_srv.SetColor.Request(r=255.0,g=0.0,b=0.0))
assert color_res.success, f"Set color error: {color_res.status_message}"

# Read the color
color_res = get_color_srv.call(reynard_srv.GetColor.Request())
assert color_res.success, f"Get color error: {color_res.status_message}"
print(f"Color: {color_res.r}, {color_res.g}, {color_res.b}")

time.sleep(1)

# Reset the color
color_res = set_color_srv.call(reynard_srv.SetColor.Request(r=0.929,g=0.49,b=0.192))
assert color_res.success, f"Set color error: {color_res.status_message}"

# Say hello
say_pub.publish(String(data="Hello, World From ROS!"))

rclpy.shutdown()