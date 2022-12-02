#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Nav2Ros2ControlInterfaceVelMsgs(Node):
    def __init__(self):
        super().__init__("nav2_ros2_ctrl_interface")
        self.vel_pub = self.create_publisher(
            Twist, "diff_cont/cmd_vel_unstamped", 10)
        self.vel_subs = self.create_subscription(
            Twist, "cmd_vel", self.callback_vel_msg, 10)
        self.get_logger().info("nav2 --> ros2_control interface velocity node has been started!")

    def callback_vel_msg(self, msg):
        #new_msg = Twist()
        #new_msg.data = msg.data
        self.vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Nav2Ros2ControlInterfaceVelMsgs()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()