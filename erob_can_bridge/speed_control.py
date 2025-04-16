#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Speed Control Example Program
This program demonstrates how to use ROS 2 topics to control motor speed
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
from std_srvs.srv import Trigger
import time
import sys

class SpeedControlNode(Node):
    def __init__(self):
        super().__init__('speed_control_node')
        
        # Create publishers
        self.enable_pub = self.create_publisher(Bool, 'motor_enable', 10)
        self.speed_pub = self.create_publisher(Int32MultiArray, 'speed_cmd_simple', 10)
        
        # Create service client
        self.init_speed_mode_client = self.create_client(Trigger, 'init_speed_mode')
        
        # Parse command line arguments
        self.motor_id = 1  # Default motor ID
        self.target_speed = 10000  # Default target speed
        
        # Parse command line arguments
        if len(sys.argv) > 1:
            try:
                self.motor_id = int(sys.argv[1])
            except ValueError:
                self.get_logger().error(f"Invalid motor ID: {sys.argv[1]}")
                
        if len(sys.argv) > 2:
            try:
                self.target_speed = int(sys.argv[2])
            except ValueError:
                self.get_logger().error(f"Invalid target speed: {sys.argv[2]}")
        
        self.get_logger().info(f"Speed Control: Motor ID={self.motor_id}, Target Speed={self.target_speed}")
        
        # Create timer, execute after 2 seconds delay
        self.timer = self.create_timer(2.0, self.send_commands)
        self.commands_sent = False
    
    def send_commands(self):
        """Send motor command sequence"""
        if self.commands_sent:
            return
            
        self.commands_sent = True
        self.timer.cancel()  # Cancel timer, execute only once
        
        # 1. Initialize speed mode
        self.get_logger().info("Initializing speed mode...")
        while not self.init_speed_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for init_speed_mode service...')
        
        request = Trigger.Request()
        future = self.init_speed_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("Speed mode initialization successful")
        else:
            self.get_logger().error(f"Speed mode initialization failed: {future.result().message}")
            self.shutdown()
            return
        
        time.sleep(1.0)
        
        # 2. Enable motor
        self.get_logger().info("Enabling motor...")
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)
        time.sleep(1.0)
        
        # 3. Send speed command
        self.get_logger().info(f"Sending speed command: Motor {self.motor_id} -> Speed {self.target_speed}")
        speed_msg = Int32MultiArray()
        speed_msg.data = [self.motor_id, self.target_speed]
        self.speed_pub.publish(speed_msg)
        
        # Exit after 10 seconds
        self.get_logger().info("Command sent, exiting in 10 seconds...")
        self.exit_timer = self.create_timer(10.0, self.shutdown)
    
    def shutdown(self):
        """Shutdown node"""
        # Stop motor
        self.get_logger().info("Stopping motor...")
        speed_msg = Int32MultiArray()
        speed_msg.data = [self.motor_id, 0]  # Set speed to 0
        self.speed_pub.publish(speed_msg)
        time.sleep(0.5)
        
        # Disable motor
        self.get_logger().info("Disabling motor...")
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        time.sleep(0.5)
        
        self.get_logger().info("Speed control completed, exiting program")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SpeedControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 