#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Position Control Example Program
This program demonstrates how to use ROS 2 topics to control motor position
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
import time
import sys

class PositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control_node')
        
        # Create publishers
        self.enable_pub = self.create_publisher(Bool, 'motor_enable', 10)
        self.position_pub = self.create_publisher(Int32MultiArray, 'position_cmd', 10)
        
        # Parse command line arguments
        self.motor_id = 1  # Default motor ID
        self.target_position = 10000  # Default target position
        
        # Parse command line arguments
        if len(sys.argv) > 1:
            try:
                self.motor_id = int(sys.argv[1])
            except ValueError:
                self.get_logger().error(f"Invalid motor ID: {sys.argv[1]}")
                
        if len(sys.argv) > 2:
            try:
                self.target_position = int(sys.argv[2])
            except ValueError:
                self.get_logger().error(f"Invalid target position: {sys.argv[2]}")
        
        self.get_logger().info(f"Position Control: Motor ID={self.motor_id}, Target Position={self.target_position}")
        
        # Create timer, execute after 2 seconds delay
        self.timer = self.create_timer(2.0, self.send_commands)
        self.commands_sent = False
    
    def send_commands(self):
        """Send motor command sequence"""
        if self.commands_sent:
            return
            
        self.commands_sent = True
        self.timer.cancel()  # Cancel timer, execute only once
        
        # 1. Enable motor
        self.get_logger().info("Enabling motor...")
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)
        time.sleep(1.0)
        
        # 2. Send position command
        self.get_logger().info(f"Sending position command: Motor {self.motor_id} -> Position {self.target_position}")
        pos_msg = Int32MultiArray()
        pos_msg.data = [self.motor_id, self.target_position]
        self.position_pub.publish(pos_msg)
        
        # Exit after 5 seconds
        self.get_logger().info("Command sent, exiting in 5 seconds...")
        self.exit_timer = self.create_timer(5.0, self.shutdown)
    
    def shutdown(self):
        """Shutdown node"""
        self.get_logger().info("Position control completed, exiting program")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PositionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()