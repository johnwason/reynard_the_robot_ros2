import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import reynard_the_robot_ros2_msgs.msg as reynard_msg
import reynard_the_robot_ros2_msgs.srv as reynard_srv
from reynard_the_robot import Reynard
import numpy as np

class ReynardTheRobotROS2Node(Node):
    def __init__(self):
        super().__init__('reynard_the_robot_ros2_node')
        self.reynard = Reynard()
        self.reynard.start()
        self.pub = self.create_publisher(reynard_msg.ReynardState, 'reynard_state', 10)
        self.say_pub = self.create_subscription(String, 'say', self.say_callback, 10)
        self.new_message_pub = self.create_publisher(String, 'new_message', 10)
        self.teleport_srv = self.create_service(reynard_srv.Teleport, 'teleport', self.teleport_callback)
        self.drive_robot_srv = self.create_service(reynard_srv.Drive, 'drive_robot', self.drive_robot_callback)
        self.drive_arm_srv = self.create_service(reynard_srv.Drive, 'drive_arm', self.drive_arm_callback)
        self.set_color_srv = self.create_service(reynard_srv.SetColor, 'set_color', self.set_color_callback)
        self.get_color_srv = self.create_service(reynard_srv.GetColor, 'get_color', self.get_color_callback)
        self.set_arm_position_srv = self.create_service(reynard_srv.SetPosition, 'set_arm_position', self.set_arm_position_callback)
        
        def _new_message_cb(_, message):
            ros_msg = String(message)
            self.new_message_pub.publish(ros_msg)

        self.reynard.new_message.connect(_new_message_cb)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ros_state = reynard_msg.ReynardState()
        ros_state.header.stamp = self.get_clock().now().to_msg()
        ros_state.time = self.reynard.time
        ros_state.robot_position = np.array(self.reynard.robot_position,dtype=np.float64)*1e-3
        ros_state.arm_position = np.deg2rad(self.reynard.arm_position)
        ros_state.robot_velocity = np.array(self.reynard.robot_velocity,dtype=np.float64)*1e-3
        ros_state.arm_velocity = np.deg2rad(self.reynard.arm_velocity)
        self.pub.publish(ros_state)

    def say_callback(self, msg):
        self.reynard.say(msg.data)

    def teleport_callback(self, req, res):
        try:
            self.reynard.teleport(req.x*1e3, req.y*1e3)
        except Exception as e:
            res.success = False
            res.status_message = str(e)
        else:
            res.success = True
        return res
    
    def drive_robot_callback(self, req, res):
        try:
            assert len(req.velocity) == 2, "Velocity must be a 2-element list"
            vel2 = np.array(req.velocity, dtype=np.float64)*1e3
            self.reynard.drive_robot(vel2[0], vel2[1], req.timeout, req.wait)
        except Exception as e:
            res.success = False
            res.status_message = str(e)
        else:
            res.success = True
        return res
    
    def set_arm_position_callback(self, req, res):
        try:
            assert len(req.target_position) == 3, "Position must be a 3-element list"
            pos = np.rad2deg(np.array(req.target_position, dtype=np.float64))
            self.reynard.set_arm_position(pos[0], pos[1], pos[2])
        except Exception as e:
            res.success = False
            res.status_message = str(e)
        else:
            res.success = True
        return res
    
    def drive_arm_callback(self, req, res):
        try:
            assert len(req.velocity) == 3, "Velocity must be a 3-element list"
            vel = np.rad2deg(np.array(req.velocity, dtype=np.float64))
            self.reynard.drive_arm(vel[0], vel[1], vel[2], req.timeout, req.wait)
        except Exception as e:
            res.success = False
            res.status_message = str(e)
        else:
            res.success = True
        return res

    def set_color_callback(self, req, res):
        try:
            self.reynard.color = (req.r, req.g, req.b)
        except Exception as e:
            res.success = False
            res.status_message = str(e)
        else:
            res.success = True
        return res

    def get_color_callback(self, req, res):
        try:
            res.r, res.g, res.b = self.reynard.color
        except Exception as e:
            res.success = False
            res.status_message = str(e)
        else:
            res.success = True
        return res
        

def main():
    rclpy.init()

    reynard_ros2 = ReynardTheRobotROS2Node()

    rclpy.spin(reynard_ros2)

    rclpy.shutdown()


if __name__ == '__main__':
    main()