#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CAN Bridge Node for ROS 2
This node bridges between ROS 2 and CAN bus for motor control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32, Bool, Int32MultiArray
from std_srvs.srv import Trigger, SetBool
import time
import threading
import json
from typing import Dict, List, Optional
import can

# Import custom CAN protocol from main.py
from erob_can_bridge.can_protocol import (
    MotorParams, MotorCommands, ControlModes, MotionModes,
    CANMotor, MotorController
)

# Remove the incorrect import
# from example_interfaces.srv import SetInt32


class CANBridgeNode(Node):
    def __init__(self):
        super().__init__('can_bridge_node')
        
        # Declare parameters
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('scan_motors_on_startup', True)
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])  # Modified default motor IDs
        
        # Get parameters
        self.can_channel = self.get_parameter('can_channel').value
        self.can_bitrate = self.get_parameter('can_bitrate').value
        self.scan_motors_on_startup = self.get_parameter('scan_motors_on_startup').value
        self.motor_ids = self.get_parameter('motor_ids').value
        
        # Initialize CAN controller
        self.controller = MotorController(
            can_channel=self.can_channel,
            bitrate=self.can_bitrate
        )
        
        # Motor objects dictionary
        self.motors = {}
        
        # Create publishers
        self.status_pub = self.create_publisher(String, 'motor_status', 10)
        self.position_pub = self.create_publisher(Int32, 'motor_position', 10)
        self.speed_pub = self.create_publisher(Int32, 'motor_speed', 10)
        self.current_pub = self.create_publisher(Float32, 'motor_current', 10)
        self.error_pub = self.create_publisher(String, 'motor_error', 10)
        
        # Create subscribers
        self.position_cmd_sub = self.create_subscription(
            Int32MultiArray, 'position_cmd', self.position_cmd_callback, 10)
        self.speed_cmd_sub = self.create_subscription(
            Int32MultiArray, 'speed_cmd', self.speed_cmd_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, 'motor_enable', self.enable_callback, 10)
        self.broadcast_position_cmd_sub = self.create_subscription(
            Int32, 'broadcast_position_cmd', self.broadcast_position_cmd_callback, 10)
        self.broadcast_speed_cmd_sub = self.create_subscription(
            Int32, 'broadcast_speed_cmd', self.broadcast_speed_cmd_callback, 10)
        self.speed_cmd_simple_sub = self.create_subscription(
            Int32MultiArray, 'speed_cmd_simple', self.speed_cmd_simple_callback, 10)
        
        # Create services
        self.scan_srv = self.create_service(
            Trigger, 'scan_motors', self.scan_motors_callback)
        self.emergency_stop_srv = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback)
        self.enable_srv = self.create_service(
            SetBool, 'set_motor_enable', self.set_enable_callback)
        self.init_position_mode_srv = self.create_service(
            Trigger, 'init_position_mode', self.init_position_mode_callback)
        self.init_speed_mode_srv = self.create_service(
            Trigger, 'init_speed_mode', self.init_speed_mode_callback)
        self.read_error_srv = self.create_service(
            Trigger, 'read_error_code', self.read_error_code_callback)
        self.clear_error_srv = self.create_service(
            Trigger, 'clear_error', self.clear_error_callback)
        self.direct_position_srv = self.create_service(
            Trigger, 'direct_position_control_service', self.direct_position_service_callback)
        
        # Create timers
        self.status_timer = self.create_timer(1.0, self.status_timer_callback)
        
        # Connect to CAN bus
        self.get_logger().info(f"Connecting to CAN bus: {self.can_channel}")
        if not self.controller.connect():
            self.get_logger().error("Failed to connect to CAN bus")
            return
        else:
            self.get_logger().info("Successfully connected to CAN bus")
        
        # Start status listener
        self.controller.start_status_listener(callback=self.status_callback)
        
        # Scan for motors if enabled
        if self.scan_motors_on_startup:
            self.scan_motors()
        else:
            # Add motors from parameter list
            for motor_id in self.motor_ids:
                self.add_motor(motor_id)
        
        self.get_logger().info("CAN Bridge Node initialized")
        
        # Add synchronous start service
        self.sync_start_srv = self.create_service(
            Trigger, 'sync_start_motors', self.sync_start_motors_callback)
        
        # Add synchronous speed setting service - using SetBool service
        # Since we don't have SetInt32 service type, we use SetBool and distinguish speed values through parameters
        self.sync_speed_srv = self.create_service(
            SetBool, 'sync_set_speed', self.sync_set_speed_callback)
    
    def add_motor(self, motor_id: int):
        """Add a motor to the controller"""
        try:
            motor = self.controller.add_motor(motor_id, f"Motor_{motor_id}")
            self.motors[motor_id] = motor
            self.get_logger().info(f"Added motor with ID: {motor_id}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to add motor {motor_id}: {e}")
            return False
    
    def scan_motors(self):
        """Scan for motors on the CAN bus"""
        try:
            self.get_logger().info("Scanning for motors on CAN bus...")
            
            # Directly use CAN commands for scanning, not relying on controller.scan_can_ids
            found_motors = []
            
            # Listener thread for receiving responses
            def listen_thread():
                start_time = time.time()
                while time.time() - start_time < 10:  # Listen for 10 seconds
                    try:
                        msg = self.controller.can_bus.recv(timeout=0.1)
                        if msg:
                            # Check if it's a status return message (0x580+)
                            if 0x580 <= msg.arbitration_id < 0x580 + 16:
                                motor_id = msg.arbitration_id - 0x580
                                if motor_id not in found_motors:
                                    found_motors.append(motor_id)
                                    self.get_logger().info(f"Found motor: ID={motor_id}")
                    except Exception as e:
                        self.get_logger().error(f"Error in listen thread: {e}")
            
            # Start the listener thread
            import threading
            listen_thread = threading.Thread(target=listen_thread)
            listen_thread.daemon = True
            listen_thread.start()
            
            # Send query commands to all possible motor IDs
            for motor_id in range(1, 15):  # Scan IDs 1-14
                self.get_logger().info(f"Querying motor ID: {motor_id}")
                
                # Send multiple commands to increase detection success rate
                commands = [
                    [0x00, 0x63],  # Read position
                    [0x00, 0x64],  # Read speed
                    [0x00, 0x66],  # Read current
                    [0x00, 0x43]   # Read CAN ID
                ]
                
                for cmd in commands:
                    try:
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=cmd,
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.2)  # Wait for response
                    except Exception as e:
                        self.get_logger().error(f"Error sending command to motor {motor_id}: {e}")
            
            # Wait for the listener thread to complete
            listen_thread.join(timeout=10)
            
            if not found_motors:
                self.get_logger().warn("No motors found during scan")
                return False
            
            self.get_logger().info(f"Found {len(found_motors)} motors: {found_motors}")
            
            # Add each found motor
            for motor_id in found_motors:
                if motor_id not in self.motors:
                    self.add_motor(motor_id)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Error scanning motors: {e}")
            return False
    
    def status_callback(self, motor_id, status, param_index=None, data_value=None, 
                       is_error=False, is_write=False, error_code=None):
        """Callback for motor status updates"""
        try:
            # Publish status update
            if motor_id in self.motors:
                # Create status message
                status_msg = String()
                status_data = {
                    'motor_id': motor_id,
                    'position': status.get('position', 0),
                    'speed': status.get('speed', 0),
                    'current': status.get('motor_current', 0),
                    'temperature': status.get('power_module_temp', 0),
                    'error_code': status.get('error_code', 0),
                    'run_status': status.get('run_status', 0),
                    'timestamp': time.time()
                }
                status_msg.data = json.dumps(status_data)
                self.status_pub.publish(status_msg)
                
                # Publish individual values
                if 'position' in status:
                    pos_msg = Int32()
                    pos_msg.data = status['position']
                    self.position_pub.publish(pos_msg)
                
                if 'speed' in status:
                    speed_msg = Int32()
                    speed_msg.data = status['speed']
                    self.speed_pub.publish(speed_msg)
                
                if 'motor_current' in status:
                    current_msg = Float32()
                    current_msg.data = float(status['motor_current']) / 1000.0  # Convert to amps
                    self.current_pub.publish(current_msg)
                
                # Publish errors
                if is_error or (status.get('error_code', 0) != 0):
                    error_msg = String()
                    error_data = {
                        'motor_id': motor_id,
                        'error_code': error_code if error_code is not None else status.get('error_code', 0),
                        'timestamp': time.time()
                    }
                    error_msg.data = json.dumps(error_data)
                    self.error_pub.publish(error_msg)
                    self.get_logger().warn(f"Motor {motor_id} error: {error_data}")
        
        except Exception as e:
            self.get_logger().error(f"Error in status callback: {e}")
    
    def status_timer_callback(self):
        """Timer callback to periodically read motor status"""
        self.get_logger().debug("Status timer triggered")
        
        if not self.motors:
            self.get_logger().warn_once("No motors available for status reading")
            return
        
        for motor_id, motor in self.motors.items():
            try:
                # Only read position, speed, and current, not error code
                self.get_logger().debug(f"Reading status for motor {motor_id}")
                motor.read_position()
                time.sleep(0.05)
                motor.read_speed()
                time.sleep(0.05)
                motor.read_current()
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"Error reading status for motor {motor_id}: {e}")
    
    def position_cmd_callback(self, msg):
        """Callback for position command messages with motor ID"""
        # Assume msg.data[0] is motor ID, msg.data[1] is target position
        if len(msg.data) < 2:
            self.get_logger().warn("Invalid position command format. Expected [motor_id, position]")
            return
        
        motor_id = msg.data[0]
        target_position = msg.data[1]
        
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            try:
                self.get_logger().info(f"Setting motor {motor_id} position to {target_position}")
                
                # Check if motor is already initialized in position control mode
                if not hasattr(self, 'initialized_motors'):
                    self.initialized_motors = {}
                
                # If motor is not initialized in position control mode, initialize it
                if motor_id not in self.initialized_motors or not self.initialized_motors.get(motor_id, {}).get('position_mode', False):
                    self.get_logger().info(f"Initializing motor {motor_id} to position control mode")
                    
                    # Use direct CAN commands to control motor
                    try:
                        # 1. Disable motor
                        self.get_logger().info(f"Disabling motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.5)
                        
                        # 2. Set control mode to position control
                        self.get_logger().info(f"Setting control mode to position for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x4E, 0x00, 0x00, 0x00, 0x03],
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.5)
                        
                        # 3. Set motion mode to absolute position mode
                        self.get_logger().info(f"Setting motion mode to absolute for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x8D, 0x00, 0x00, 0x00, 0x01],
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.5)
                        
                        # 4. Set acceleration
                        self.get_logger().info(f"Setting acceleration for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x88, 0x00, 0x01, 0x86, 0xA0],  # 100000
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.5)
                        
                        # 5. Set deceleration
                        self.get_logger().info(f"Setting deceleration for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x89, 0x00, 0x01, 0x86, 0xA0],  # 100000
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.5)
                        
                        # 6. Set target speed
                        self.get_logger().info(f"Setting target speed for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x8A, 0x00, 0x01, 0x86, 0xA0],  # 100000
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.5)
                        
                        # 7. Enable motor
                        self.get_logger().info(f"Enabling motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x01],
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.5)
                        
                        # Mark motor as initialized in position control mode
                        if motor_id not in self.initialized_motors:
                            self.initialized_motors[motor_id] = {}
                        self.initialized_motors[motor_id]['position_mode'] = True
                        
                        self.get_logger().info(f"Motor {motor_id} initialized to position control mode")
                    except Exception as e:
                        self.get_logger().error(f"Error initializing motor {motor_id}: {e}")
                        return
                else:
                    self.get_logger().info(f"Motor {motor_id} already in position control mode, skipping initialization")
                
                # Directly set target position and start motion
                try:
                    # Set target position
                    self.get_logger().info(f"Setting target position to {target_position} for motor {motor_id}")
                    pos_bytes = [(target_position >> 24) & 0xFF, 
                                 (target_position >> 16) & 0xFF, 
                                 (target_position >> 8) & 0xFF, 
                                 target_position & 0xFF]
                    msg = can.Message(
                        arbitration_id=0x640 + motor_id,
                        data=[0x00, 0x86] + pos_bytes,
                        is_extended_id=False
                    )
                    self.controller.can_bus.send(msg)
                    time.sleep(0.1)  # 减少延迟
                    
                    # Start motion
                    self.get_logger().info(f"Starting motion for motor {motor_id}")
                    msg = can.Message(
                        arbitration_id=0x640 + motor_id,
                        data=[0x00, 0x83],
                        is_extended_id=False
                    )
                    self.controller.can_bus.send(msg)
                    
                    self.get_logger().info(f"Successfully sent position command for motor {motor_id}")
                except Exception as e:
                    self.get_logger().error(f"Error in direct position control for motor {motor_id}: {e}")
                    import traceback
                    traceback.print_exc()
                    
            except Exception as e:
                self.get_logger().error(f"Error setting position for motor {motor_id}: {e}")
                import traceback
                traceback.print_exc()
        else:
            self.get_logger().warn(f"Motor {motor_id} not found")
    
    def speed_cmd_callback(self, msg):
        """Callback for speed command messages with motor ID"""
        # Assume msg.data[0] is motor ID, msg.data[1] is target speed
        if len(msg.data) < 2:
            self.get_logger().warn("Invalid speed command format. Expected [motor_id, speed]")
            return
        
        motor_id = msg.data[0]
        target_speed = msg.data[1]
        
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            try:
                self.get_logger().info(f"Setting motor {motor_id} speed to {target_speed}")
                
                # Check if motor is already initialized in speed control mode
                if not hasattr(self, 'initialized_motors'):
                    self.initialized_motors = {}
                
                # If motor is not initialized in speed control mode, initialize it
                if motor_id not in self.initialized_motors or not self.initialized_motors.get(motor_id, {}).get('speed_mode', False):
                    self.get_logger().info(f"Initializing motor {motor_id} to speed control mode")
                    
                    # Use direct CAN commands to control motor
                    try:
                        # 1. Disable motor
                        self.get_logger().info(f"Disabling motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.2)
                        
                        # 2. Set control mode to speed control
                        self.get_logger().info(f"Setting control mode to speed for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x4E, 0x00, 0x00, 0x00, 0x02],  # 速度控制模式
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.2)
                        
                        # 3. Set acceleration
                        self.get_logger().info(f"Setting acceleration for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x88, 0x00, 0x01, 0x86, 0xA0],  # 100000
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.2)
                        
                        # 4. Set deceleration
                        self.get_logger().info(f"Setting deceleration for motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x00, 0x89, 0x00, 0x01, 0x86, 0xA0],  # 100000
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.2)
                        
                        # 5. Enable motor
                        self.get_logger().info(f"Enabling motor {motor_id}")
                        msg = can.Message(
                            arbitration_id=0x640 + motor_id,
                            data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x01],
                            is_extended_id=False
                        )
                        self.controller.can_bus.send(msg)
                        time.sleep(0.2)
                        
                        # Mark motor as initialized in speed control mode
                        if motor_id not in self.initialized_motors:
                            self.initialized_motors[motor_id] = {}
                        self.initialized_motors[motor_id]['speed_mode'] = True
                        
                        self.get_logger().info(f"Motor {motor_id} initialized to speed control mode")
                    except Exception as e:
                        self.get_logger().error(f"Error initializing motor {motor_id}: {e}")
                        return
                else:
                    self.get_logger().info(f"Motor {motor_id} already in speed control mode, skipping initialization")
                
                # Directly set target speed and start motion
                try:
                    # Set target speed
                    self.get_logger().info(f"Setting target speed to {target_speed} for motor {motor_id}")
                    speed_bytes = [(target_speed >> 24) & 0xFF, 
                                  (target_speed >> 16) & 0xFF, 
                                  (target_speed >> 8) & 0xFF, 
                                  target_speed & 0xFF]
                    msg = can.Message(
                        arbitration_id=0x640 + motor_id,
                        data=[0x00, 0x8A] + speed_bytes,
                        is_extended_id=False
                    )
                    self.controller.can_bus.send(msg)
                    time.sleep(0.1)  # 减少延迟
                    
                    # Start motion
                    self.get_logger().info(f"Starting motion for motor {motor_id}")
                    msg = can.Message(
                        arbitration_id=0x640 + motor_id,
                        data=[0x00, 0x83],
                        is_extended_id=False
                    )
                    self.controller.can_bus.send(msg)
                    
                    self.get_logger().info(f"Successfully sent speed command for motor {motor_id}")
                except Exception as e:
                    self.get_logger().error(f"Error in direct speed control for motor {motor_id}: {e}")
                    import traceback
                    traceback.print_exc()
                    
            except Exception as e:
                self.get_logger().error(f"Error setting speed for motor {motor_id}: {e}")
                import traceback
                traceback.print_exc()
        else:
            self.get_logger().warn(f"Motor {motor_id} not found")
    
    def enable_callback(self, msg):
        """Callback for enable/disable messages"""
        # Apply to all motors
        for motor_id, motor in self.motors.items():
            try:
                if msg.data:
                    self.get_logger().info(f"Enabling motor {motor_id}")
                    motor.enable()
                else:
                    self.get_logger().info(f"Disabling motor {motor_id}")
                    motor.disable()
            except Exception as e:
                self.get_logger().error(f"Error setting enable state for motor {motor_id}: {e}")
    
    def broadcast_position_cmd_callback(self, msg):
        """Send position command to all motors"""
        target_position = msg.data
        self.get_logger().info(f"Broadcasting position command {target_position} to all motors")
        
        # First ensure all motors are in position control mode
        for motor_id, motor in self.motors.items():
            try:
                # Ensure motor is in position control mode
                if not hasattr(self, 'initialized_motors') or motor_id not in self.initialized_motors or not self.initialized_motors.get(motor_id, {}).get('position_mode', False):
                    self.get_logger().info(f"Motor {motor_id} not initialized for position control, initializing now")
                    # Initialize motor to position control mode
                    self._init_motor_position_mode(motor_id)
                    time.sleep(0.5)  # Give motor some time to complete initialization
            except Exception as e:
                self.get_logger().error(f"Error initializing motor {motor_id}: {e}")
        
        # Set target position for all motors and start immediately
        # We don't use broadcast ID here, instead we set each motor individually and start it immediately
        for motor_id, motor in self.motors.items():
            try:
                # Set target position
                self.get_logger().info(f"Setting target position {target_position} for motor {motor_id}")
                pos_bytes = [(target_position >> 24) & 0xFF, 
                             (target_position >> 16) & 0xFF, 
                             (target_position >> 8) & 0xFF, 
                             target_position & 0xFF]
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x00, 0x86] + pos_bytes,
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                time.sleep(0.05)  # Short delay to ensure command is processed
                
                # Immediately start the motor
                self.get_logger().info(f"Starting motion for motor {motor_id}")
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x00, 0x83],
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                time.sleep(0.01)  # 极短延迟，尽量同时启动
            except Exception as e:
                self.get_logger().error(f"Error setting position and starting motor {motor_id}: {e}")
        
        self.get_logger().info("All motors have been commanded to move to the target position")
    
    def _init_motor_position_mode(self, motor_id):
        """初始化电机为位置控制模式的辅助函数"""
        try:
            # 1. Disable motor
            self.get_logger().info(f"Disabling motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 2. Set control mode to position control
            self.get_logger().info(f"Setting control mode to position for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x4E, 0x00, 0x00, 0x00, 0x03],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 3. Set motion mode to absolute position mode
            self.get_logger().info(f"Setting motion mode to absolute for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x8D, 0x00, 0x00, 0x00, 0x01],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 4. Set acceleration
            self.get_logger().info(f"Setting acceleration for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x88, 0x00, 0x01, 0x86, 0xA0],  # 100000
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 5. Set deceleration
            self.get_logger().info(f"Setting deceleration for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x89, 0x00, 0x01, 0x86, 0xA0],  # 100000
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 6. Set target speed
            self.get_logger().info(f"Setting target speed for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x8A, 0x00, 0x01, 0x86, 0xA0],  # 100000
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 7. Enable motor
            self.get_logger().info(f"Enabling motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x01],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # Mark motor as initialized in position control mode
            if not hasattr(self, 'initialized_motors'):
                self.initialized_motors = {}
            if motor_id not in self.initialized_motors:
                self.initialized_motors[motor_id] = {}
            self.initialized_motors[motor_id]['position_mode'] = True
            
            self.get_logger().info(f"Motor {motor_id} initialized to position control mode")
            return True
        except Exception as e:
            self.get_logger().error(f"Error initializing motor {motor_id}: {e}")
            return False
    
    def scan_motors_callback(self, request, response):
        """Service callback to scan for motors"""
        success = self.scan_motors()
        response.success = success
        response.message = f"Found {len(self.motors)} motors" if success else "Failed to scan motors"
        return response
    
    def emergency_stop_callback(self, request, response):
        """Service callback for emergency stop"""
        try:
            self.get_logger().warn("Emergency stop triggered")
            self.controller.emergency_stop()
            response.success = True
            response.message = "Emergency stop executed"
        except Exception as e:
            self.get_logger().error(f"Error during emergency stop: {e}")
            response.success = False
            response.message = f"Emergency stop failed: {e}"
        return response
    
    def set_enable_callback(self, request, response):
        """Service callback to enable/disable motors"""
        try:
            # Apply to all motors
            for motor_id, motor in self.motors.items():
                if request.data:
                    self.get_logger().info(f"Enabling motor {motor_id}")
                    motor.enable()
                else:
                    self.get_logger().info(f"Disabling motor {motor_id}")
                    motor.disable()
            
            response.success = True
            response.message = "Motors enabled" if request.data else "Motors disabled"
        except Exception as e:
            self.get_logger().error(f"Error setting enable state: {e}")
            response.success = False
            response.message = f"Failed to set enable state: {e}"
        return response
    
    def init_position_mode_callback(self, request, response):
        """Initialize all motors to position control mode"""
        success = True
        try:
            self.get_logger().info("Initializing all motors to position control mode")
            
            # First ensure all motors are disabled
            self.get_logger().info("Ensuring all motors are disabled")
            for motor_id, motor in self.motors.items():
                try:
                    motor.disable()
                except Exception as e:
                    self.get_logger().error(f"Failed to disable motor {motor_id}: {e}")
            
            # Wait long enough to ensure all motors are disabled
            time.sleep(0.5)
            
            # Simplify initialization process, only set control mode and acceleration/deceleration/speed parameters
            for motor_id, motor in self.motors.items():
                try:
                    self.get_logger().info(f"Setting position control parameters for motor {motor_id}")
                    
                    # Set control mode to position control
                    if not motor.controller.set_parameter(motor_id, MotorParams.CONTROL_MODE, ControlModes.POSITION):
                        self.get_logger().warn(f"Failed to set control mode for motor {motor_id}")
                        success = False
                        continue
                    time.sleep(0.2)
                    
                    # Set acceleration
                    if not motor.controller.set_parameter(motor_id, MotorParams.ACCELERATION, 50000):
                        self.get_logger().warn(f"Failed to set acceleration for motor {motor_id}")
                        # Don't break, continue trying to set other parameters
                    time.sleep(0.2)
                    
                    # Set deceleration
                    if not motor.controller.set_parameter(motor_id, MotorParams.DECELERATION, 50000):
                        self.get_logger().warn(f"Failed to set deceleration for motor {motor_id}")
                        # Don't break, continue trying to set other parameters
                    time.sleep(0.2)
                    
                    # Set target speed
                    if not motor.controller.set_parameter(motor_id, MotorParams.TARGET_SPEED, 50000):
                        self.get_logger().warn(f"Failed to set target speed for motor {motor_id}")
                        # Don't break, continue trying to set other parameters
                    time.sleep(0.2)
                    
                    # Enable motor
                    if not motor.controller.set_parameter(motor_id, MotorParams.MOTOR_ENABLE, 1):
                        self.get_logger().warn(f"Failed to enable motor {motor_id}")
                        success = False
                        continue
                    time.sleep(0.2)
                    
                    # Mark motor as initialized
                    if not hasattr(self, 'initialized_motors'):
                        self.initialized_motors = {}
                    if motor_id not in self.initialized_motors:
                        self.initialized_motors[motor_id] = {}
                    self.initialized_motors[motor_id]['position_mode'] = True
                    
                    self.get_logger().info(f"Motor {motor_id} position control parameters set successfully")
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize motor {motor_id} to position mode: {e}")
                    success = False
            
            # Return response
            response.success = success
            response.message = "All motors initialized to position control mode" if success else "Some motors failed to initialize"
            return response
        except Exception as e:
            self.get_logger().error(f"Error in init_position_mode: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def init_speed_mode_callback(self, request, response):
        """Initialize all motors to speed control mode"""
        success = True
        try:
            self.get_logger().info("Initializing all motors to speed control mode")
            
            # First ensure all motors are disabled
            self.get_logger().info("Ensuring all motors are disabled")
            for motor_id, motor in self.motors.items():
                try:
                    motor.disable()
                except Exception as e:
                    self.get_logger().error(f"Failed to disable motor {motor_id}: {e}")
            
            # Wait long enough to ensure all motors are disabled
            time.sleep(0.5)
            
            # Initialize all motors to speed control mode
            for motor_id, motor in self.motors.items():
                try:
                    self.get_logger().info(f"Setting speed control parameters for motor {motor_id}")
                    
                    # Set control mode to speed control
                    if not motor.controller.set_parameter(motor_id, MotorParams.CONTROL_MODE, ControlModes.SPEED):
                        self.get_logger().warn(f"Failed to set control mode for motor {motor_id}")
                        success = False
                        continue
                    time.sleep(0.2)
                    
                    # Set acceleration
                    if not motor.controller.set_parameter(motor_id, MotorParams.ACCELERATION, 50000):
                        self.get_logger().warn(f"Failed to set acceleration for motor {motor_id}")
                        # Don't break, continue trying to set other parameters
                    time.sleep(0.2)
                    
                    # Set deceleration
                    if not motor.controller.set_parameter(motor_id, MotorParams.DECELERATION, 50000):
                        self.get_logger().warn(f"Failed to set deceleration for motor {motor_id}")
                        # Don't break, continue trying to set other parameters
                    time.sleep(0.2)
                    
                    # Enable motor
                    if not motor.controller.set_parameter(motor_id, MotorParams.MOTOR_ENABLE, 1):
                        self.get_logger().warn(f"Failed to enable motor {motor_id}")
                        success = False
                        continue
                    time.sleep(0.2)
                    
                    # Mark motor as initialized
                    if not hasattr(self, 'initialized_motors'):
                        self.initialized_motors = {}
                    if motor_id not in self.initialized_motors:
                        self.initialized_motors[motor_id] = {}
                    self.initialized_motors[motor_id]['speed_mode'] = True
                    
                    self.get_logger().info(f"Motor {motor_id} speed control parameters set successfully")
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize motor {motor_id} to speed mode: {e}")
                    success = False
            
            # Return response
            response.success = success
            response.message = "All motors initialized to speed control mode" if success else "Some motors failed to initialize"
            return response
        except Exception as e:
            self.get_logger().error(f"Error in init_speed_mode: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def setup_speed_mode_complete(self, target_speed: int = 10000) -> bool:
        """完整设置速度模式的函数"""
        success = True
        
        # 1-7步与之前相同...
        
        # 8. 重要：开始运动命令
        success &= self.start_motion()
        time.sleep(0.1)
        
        return success
    
    def speed_cmd_simple_callback(self, msg):
        """简化的速度控制命令回调，格式为[motor_id, speed]"""
        if len(msg.data) < 2:
            self.get_logger().warn("Invalid speed command format. Expected [motor_id, speed]")
            return
        
        motor_id = msg.data[0]
        target_speed = msg.data[1]
        
        if motor_id in self.motors:
            try:
                self.get_logger().info(f"Setting motor {motor_id} speed to {target_speed} (simple mode)")
                
                # 检查电机是否已初始化为速度控制模式
                if not hasattr(self, 'initialized_motors') or motor_id not in self.initialized_motors or not self.initialized_motors.get(motor_id, {}).get('speed_mode', False):
                    self.get_logger().info(f"Motor {motor_id} not initialized for speed control, initializing now")
                    self._init_motor_speed_mode(motor_id)
                
                # 设置目标速度 - 使用01 FE命令
                speed_bytes = [(target_speed >> 24) & 0xFF, 
                              (target_speed >> 16) & 0xFF, 
                              (target_speed >> 8) & 0xFF, 
                              target_speed & 0xFF]
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x01, 0xFE] + speed_bytes,  # 使用01 FE命令
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                time.sleep(0.05)
                
                # 启动运动
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x00, 0x83],  # 启动运动命令
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                
                self.get_logger().info(f"Successfully sent simple speed command for motor {motor_id}")
            except Exception as e:
                self.get_logger().error(f"Error in simple speed control for motor {motor_id}: {e}")
        else:
            self.get_logger().warn(f"Motor {motor_id} not found")
    
    def read_error_code_callback(self, request, response):
        """Service callback to read error codes from all motors"""
        try:
            error_info = {}
            for motor_id, motor in self.motors.items():
                # 读取错误代码
                motor.read_error_code()
                time.sleep(0.1)
                
                # 获取错误代码
                error_code = motor.get_error_code()
                error_info[motor_id] = error_code
                
                # 如果有错误，记录并发布
                if error_code != 0:
                    self.get_logger().warn(f"Motor {motor_id} error code: 0x{error_code:04X}")
                    
                    # 发布错误信息
                    error_msg = String()
                    error_data = {
                        'motor_id': motor_id,
                        'error_code': error_code,
                        'timestamp': time.time()
                    }
                    error_msg.data = json.dumps(error_data)
                    self.error_pub.publish(error_msg)
            
            # 返回响应
            response.success = True
            response.message = json.dumps(error_info)
            return response
        except Exception as e:
            self.get_logger().error(f"Error reading error codes: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def clear_error_callback(self, request, response):
        """Service callback to clear errors from all motors"""
        try:
            # 首先确保所有电机都处于失能状态
            self.get_logger().info("Disabling all motors to clear errors")
            for motor_id, motor in self.motors.items():
                try:
                    motor.disable()
                except Exception as e:
                    self.get_logger().error(f"Failed to disable motor {motor_id}: {e}")
            
            # 等待足够长的时间，确保所有电机都已失能
            time.sleep(1.0)
            
            # 读取所有电机的错误代码
            error_states = {}
            for motor_id, motor in self.motors.items():
                try:
                    motor.read_error_code()
                    time.sleep(0.2)
                    error_code = motor.get_error_code()
                    error_states[motor_id] = error_code
                    
                    if error_code != 0:
                        self.get_logger().info(f"Motor {motor_id} has error code: 0x{error_code:04X}")
                except Exception as e:
                    self.get_logger().error(f"Failed to read error code for motor {motor_id}: {e}")
            
            # 等待一段时间
            time.sleep(1.0)
            
            # 简单地保持电机失能状态，不要频繁切换使能状态
            # 在某些电机控制器中，错误会在电机失能一段时间后自动清除
            
            # 再次读取错误代码，检查是否已清除
            cleared_count = 0
            for motor_id, motor in self.motors.items():
                try:
                    motor.read_error_code()
                    time.sleep(0.2)
                    new_error_code = motor.get_error_code()
                    
                    if new_error_code == 0:
                        if error_states.get(motor_id, 0) != 0:
                            self.get_logger().info(f"Successfully cleared error for motor {motor_id}")
                        cleared_count += 1
                    else:
                        self.get_logger().warn(f"Motor {motor_id} still has error code: 0x{new_error_code:04X}")
                except Exception as e:
                    self.get_logger().error(f"Failed to read error code for motor {motor_id}: {e}")
            
            # 返回响应
            response.success = True
            response.message = f"Cleared errors for {cleared_count}/{len(self.motors)} motors"
            return response
        except Exception as e:
            self.get_logger().error(f"Error clearing errors: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def direct_position_service_callback(self, request, response):
        """Service callback to control motor position using direct CAN commands"""
        try:
            # 从请求中获取电机ID和位置
            # 这里我们使用默认值，您可以修改为从请求参数中获取
            motor_id = 3  # 默认电机ID
            position = 10000  # 默认位置
            
            self.get_logger().info(f"Direct position control service: motor {motor_id}, position {position}")
            
            # 1. 设置控制模式为位置控制 (00 4E 00 00 00 03)
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x4E, 0x00, 0x00, 0x00, 0x03],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 2. 设置运动模式为绝对位置模式 (00 8D 00 00 00 01)
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x8D, 0x00, 0x00, 0x00, 0x01],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 3. 设置加速度 (00 88 00 00 27 10)
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x88, 0x00, 0x00, 0x27, 0x10],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 4. 设置减速度 (00 89 00 00 27 10)
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x89, 0x00, 0x00, 0x27, 0x10],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 5. 设置目标速度 (00 8A 00 00 27 10)
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x8A, 0x00, 0x00, 0x27, 0x10],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 6. 电机使能 (01 00 00 00 00 01)
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x01],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 7. 设置相对位置为 0 (00 87 00 00 00 00)
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x87, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 8. 设置目标绝对位置
            pos_bytes = [(position >> 24) & 0xFF, (position >> 16) & 0xFF, 
                         (position >> 8) & 0xFF, position & 0xFF]
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x86] + pos_bytes,
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 9. 开始运动
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x83],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            
            response.success = True
            response.message = f"Direct position control initiated for motor {motor_id}"
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in direct position control service: {e}")
            import traceback
            traceback.print_exc()
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def broadcast_speed_cmd_callback(self, msg):
        """Send speed command to all motors"""
        target_speed = msg.data
        self.get_logger().info(f"Broadcasting speed command {target_speed} to all motors")
        
        # 首先确保所有电机都处于速度控制模式
        for motor_id, motor in self.motors.items():
            try:
                # 检查电机是否已初始化为速度控制模式
                if not hasattr(self, 'initialized_motors') or motor_id not in self.initialized_motors or not self.initialized_motors.get(motor_id, {}).get('speed_mode', False):
                    self.get_logger().info(f"Motor {motor_id} not initialized for speed control, initializing now")
                    # 初始化电机为速度控制模式
                    self._init_motor_speed_mode(motor_id)
                    time.sleep(0.5)  # 给电机一些时间完成初始化
            except Exception as e:
                self.get_logger().error(f"Error initializing motor {motor_id}: {e}")
        
        # 设置所有电机的目标速度
        for motor_id, motor in self.motors.items():
            try:
                # 设置目标速度 - 使用01 FE命令而不是00 8A
                self.get_logger().info(f"Setting target speed {target_speed} for motor {motor_id}")
                speed_bytes = [(target_speed >> 24) & 0xFF, 
                              (target_speed >> 16) & 0xFF, 
                              (target_speed >> 8) & 0xFF, 
                              target_speed & 0xFF]
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x01, 0xFE] + speed_bytes,  # 使用01 FE命令
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                time.sleep(0.05)  # 短暂延迟确保命令被处理
            except Exception as e:
                self.get_logger().error(f"Error setting speed for motor {motor_id}: {e}")
        
        # 短暂延迟后启动所有电机
        time.sleep(0.1)
        
        # 立即启动所有电机
        for motor_id, motor in self.motors.items():
            try:
                self.get_logger().info(f"Starting motion for motor {motor_id}")
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x00, 0x83],  # 启动运动命令
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                time.sleep(0.001)  # 极短延迟，尽量同时启动
            except Exception as e:
                self.get_logger().error(f"Error starting motor {motor_id}: {e}")
        
        self.get_logger().info("All motors have been commanded to move at the target speed")
    
    def _init_motor_speed_mode(self, motor_id):
        """初始化电机为速度控制模式的辅助函数，按照标准流程设置"""
        try:
            # 1. 设置控制模式为速度控制 (00 4E 00 00 00 02)
            self.get_logger().info(f"Setting control mode to speed for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x4E, 0x00, 0x00, 0x00, 0x02],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 2. 设置控制源为不使用 (01 12 00 00 00 00)
            self.get_logger().info(f"Setting control source for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x01, 0x12, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 3. 设置模拟量类型为内部使用 (01 FD 00 00 00 00)
            self.get_logger().info(f"Setting analog type for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x01, 0xFD, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 4. 设置加速度 (00 88 00 00 75 30) - 30000 count/s²
            self.get_logger().info(f"Setting acceleration for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x88, 0x00, 0x00, 0x75, 0x30],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 5. 设置减速度 (00 89 00 00 75 30) - 30000 count/s²
            self.get_logger().info(f"Setting deceleration for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x00, 0x89, 0x00, 0x00, 0x75, 0x30],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 6. 设置目标速度值 (01 FE 00 00 27 10) - 10000 count/s
            self.get_logger().info(f"Setting initial target speed for motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x01, 0xFE, 0x00, 0x00, 0x27, 0x10],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 7. 电机使能 (01 00 00 00 00 01)
            self.get_logger().info(f"Enabling motor {motor_id}")
            msg = can.Message(
                arbitration_id=0x640 + motor_id,
                data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x01],
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            time.sleep(0.2)
            
            # 标记电机已初始化为速度控制模式
            if not hasattr(self, 'initialized_motors'):
                self.initialized_motors = {}
            if motor_id not in self.initialized_motors:
                self.initialized_motors[motor_id] = {}
            self.initialized_motors[motor_id]['speed_mode'] = True
            
            self.get_logger().info(f"Motor {motor_id} initialized to speed control mode")
            return True
        except Exception as e:
            self.get_logger().error(f"Error initializing motor {motor_id}: {e}")
            return False
    
    def sync_start_motors_callback(self, request, response):
        """同步启动所有电机"""
        try:
            self.get_logger().info("Synchronously starting all motors")
            
            # First ensure all motors have been set to target position or speed
            # Then send start command in a very short time
            for motor_id in self.motors:
                try:
                    msg = can.Message(
                        arbitration_id=0x640 + motor_id,
                        data=[0x00, 0x83],  # Start motion command
                        is_extended_id=False
                    )
                    self.controller.can_bus.send(msg)
                    time.sleep(0.001)  # Short delay to ensure all speed setting commands are processed
                except Exception as e:
                    self.get_logger().error(f"Failed to start motor {motor_id}: {e}")
                    response.success = False
                    response.message = f"Failed to start some motors: {e}"
                    return response
            
            response.success = True
            response.message = "All motors started successfully"
            return response
        except Exception as e:
            self.get_logger().error(f"Error in sync_start_motors: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def sync_set_speed_callback(self, request, response):
        """同步设置所有电机的速度 - 使用SetBool服务，速度值通过参数传递"""
        # Get speed value from request - we use a fixed value here, because SetBool doesn't have an Int32 field
        target_speed = 10000  # Default speed value
        
        # If request.data is True, use positive speed, otherwise use negative speed
        if not request.data:
            target_speed = -target_speed
        
        try:
            self.get_logger().info(f"Setting all motors to speed {target_speed}")
            
            # First ensure all motors are in speed control mode
            for motor_id, motor in self.motors.items():
                if not hasattr(self, 'initialized_motors') or motor_id not in self.initialized_motors or not self.initialized_motors.get(motor_id, {}).get('speed_mode', False):
                    self.get_logger().info(f"Motor {motor_id} not initialized for speed control, initializing now")
                    self._init_motor_speed_mode(motor_id)
            
            # Set target speed for all motors - using 01 FE command
            for motor_id in self.motors:
                # Convert speed value to bytes
                speed_bytes = [(target_speed >> 24) & 0xFF, 
                              (target_speed >> 16) & 0xFF, 
                              (target_speed >> 8) & 0xFF, 
                              target_speed & 0xFF]
                
                # Use 01 FE command to set target speed
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x01, 0xFE] + speed_bytes,
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                self.get_logger().info(f"Set target speed {target_speed} for motor {motor_id}")
                time.sleep(0.05)
            
            # Important: Send start command to make new speed settings effective
            time.sleep(0.1)  # Short delay to ensure all speed setting commands are processed
            
            for motor_id in self.motors:
                # Send start command
                msg = can.Message(
                    arbitration_id=0x640 + motor_id,
                    data=[0x00, 0x83],  # Start motion command
                    is_extended_id=False
                )
                self.controller.can_bus.send(msg)
                self.get_logger().info(f"Started motion for motor {motor_id}")
                time.sleep(0.001)  # Short delay to ensure all speed setting commands are processed
            
            response.success = True
            response.message = f"All motors set to speed {target_speed} and started"
            return response
        except Exception as e:
            self.get_logger().error(f"Error in sync_set_speed: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def destroy_node(self):
        """Clean up when node is shut down"""
        self.get_logger().info("Shutting down CAN Bridge Node")
        
        # Stop all motors
        try:
            self.controller.emergency_stop()
        except Exception as e:
            self.get_logger().error(f"Error during emergency stop: {e}")
        
        # Disconnect from CAN bus
        try:
            self.controller.disconnect()
        except Exception as e:
            self.get_logger().error(f"Error disconnecting from CAN bus: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CANBridgeNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in CAN Bridge Node: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

