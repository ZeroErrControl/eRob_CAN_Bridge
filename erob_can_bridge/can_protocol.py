#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CAN Bus Motor Driver
This program implements motor control based on a specific CAN protocol, including velocity control, position control, and status monitoring
"""

import time
import struct
import threading
from typing import Dict, List, Optional, Tuple, Union, Callable

try:
    import can
except ImportError:
    print("Please install python-can library: pip install python-can")
    exit(1)

# =============================================================================
# Parameter Definitions
# =============================================================================

# CAN Command Base IDs
CAN_BASE_CMD_ID = 0x640       # Base ID for sending read data commands (640+)
CAN_BASE_STATUS_ID = 0x5C0    # Base ID for receiving read data (5C0+)

# Status Codes
CAN_STATUS_SUCCESS = 0x3E     # Success status code
CAN_STATUS_ERROR = 0x80       # Error status code

# Parameter Index Definitions (based on address table)
class MotorParams:
    """Motor parameter index constants"""
    # Read Parameters
    ACTUAL_POSITION = 0x02      # Actual position
    ACTUAL_SPEED = 0x05         # Actual speed
    MOTOR_CURRENT = 0x08        # Actual motor current
    U_PHASE_CURRENT = 0x09      # U phase current
    V_PHASE_CURRENT = 0x0A      # V phase current
    U_PHASE_VOLTAGE = 0x0D      # U phase voltage
    V_PHASE_VOLTAGE = 0x0E      # V phase voltage
    W_PHASE_VOLTAGE = 0x0F      # W phase voltage
    ERROR_CODE = 0x1F           # Error code
    RUN_STATUS = 0x20           # Running status
    DIGITAL_INPUT = 0x22        # Digital input
    ANALOG_INPUT = 0x23         # Analog input
    DC_BUS_VOLTAGE = 0x24       # DC bus voltage
    POWER_MODULE_TEMP = 0x26    # Power module temperature
    CONTINUOUS_CURRENT = 0x33   # Continuous current
    PEAK_CURRENT = 0x34         # Peak current
    PEAK_CURRENT_TIME = 0x35    # Peak current duration
    CAN_ID = 0x43               # CAN address
    BYTES_RATE = 0x44           # Byte rate
    
    # Write Parameters
    CONTROL_MODE = 0x4E         # Control mode
    MAX_SPEED = 0x50            # Maximum speed
    MIN_POSITION = 0x52         # Minimum position
    MAX_POSITION = 0x53         # Maximum position
    MAX_POSITION_ERROR = 0x54   # Maximum position error
    MAX_SPEED_ERROR = 0x55      # Maximum speed error
    STALL_CURRENT = 0x56        # Stall current
    STALL_SPEED = 0x57          # Stall speed
    STALL_TIME = 0x58           # Stall time
    MAX_PHASE_CURRENT = 0x62    # Maximum phase current
    MAX_SHUTDOWN_CURRENT = 0x63 # Maximum shutdown current
    POSITION_LOOP_GAIN = 0x64   # Position loop gain
    SPEED_LOOP_GAIN = 0x66      # Speed loop gain
    SPEED_LOOP_INTEGRAL = 0x67  # Speed loop integral
    MOTION_MODE = 0x8D          # Motion mode
    ACCELERATION = 0x88         # Acceleration
    DECELERATION = 0x89         # Deceleration
    TARGET_SPEED = 0x8A         # Target speed
    TARGET_POSITION = 0x86      # Target position
    RELATIVE_POSITION = 0x87    # Relative position amount
    MOTOR_ENABLE = 0x100        # Motor enable


# Command Index Definitions
class MotorCommands:
    """Motor command index constants"""
    START_MOTION = 0x83         # Start motion
    STOP_MOTION = 0x84          # Stop motion
    SAVE_PARAMETERS = 0xE8      # Save parameters
    RELEASE_BRAKE = 0x014F      # Release brake
    MOTOR_ENABLE = 0x100        # Motor enable

    

# Control Mode Definitions
class ControlModes:
    """Control mode constants"""
    TORQUE = 1                  # Torque control
    SPEED = 2                   # Speed control
    POSITION = 3                # Position control

# Motion Mode Definitions
class MotionModes:
    """Motion mode constants"""
    RELATIVE = 0                # Relative position mode
    ABSOLUTE = 1                # Absolute position mode

# Parameter Name Mapping
PARAM_NAMES = {
    MotorParams.ACTUAL_POSITION: "Actual Position",
    MotorParams.ACTUAL_SPEED: "Actual Speed",
    MotorParams.MOTOR_CURRENT: "Motor Current",
    MotorParams.U_PHASE_CURRENT: "U Phase Current",
    MotorParams.V_PHASE_CURRENT: "V Phase Current",
    MotorParams.U_PHASE_VOLTAGE: "U Phase Voltage",
    MotorParams.V_PHASE_VOLTAGE: "V Phase Voltage",
    MotorParams.W_PHASE_VOLTAGE: "W Phase Voltage",
    MotorParams.ERROR_CODE: "Error Code",
    MotorParams.RUN_STATUS: "Running Status",
    MotorParams.DIGITAL_INPUT: "Digital Input",
    MotorParams.ANALOG_INPUT: "Analog Input",
    MotorParams.DC_BUS_VOLTAGE: "DC Bus Voltage",
    MotorParams.POWER_MODULE_TEMP: "Power Module Temperature",
    MotorParams.CONTINUOUS_CURRENT: "Continuous Current",
    MotorParams.PEAK_CURRENT: "Peak Current",
    MotorParams.PEAK_CURRENT_TIME: "Peak Current Duration",
    MotorParams.CAN_ID: "CAN Address",
    MotorParams.CONTROL_MODE: "Control Mode",
    MotorParams.MOTION_MODE: "Motion Mode",
    MotorParams.ACCELERATION: "Acceleration",
    MotorParams.DECELERATION: "Deceleration",
    MotorParams.TARGET_SPEED: "Target Speed",
    MotorParams.TARGET_POSITION: "Target Position",
    MotorParams.RELATIVE_POSITION: "Relative Position Amount",
    MotorCommands.MOTOR_ENABLE: "Motor Enable"
}

# =============================================================================
# 电机类
# =============================================================================

class CANMotor:
    """Motor encapsulation class, providing simple control interface"""
    
    def __init__(self, controller, motor_id: int, name: str = None):
        """
        Initialize motor object
        
        Parameters:
            controller: Motor controller instance
            motor_id: Motor ID
            name: Motor name
        """
        self.controller = controller
        self.id = motor_id
        self.name = name or f"Motor_{motor_id}"
        self.status = {}
    
    # -------------------------------------------------------------------------
    # Basic Control Methods
    # -------------------------------------------------------------------------
    
    def enable(self) -> bool:
        """Enable motor"""
        return self.controller.set_parameter(self.id, MotorParams.MOTOR_ENABLE, 1)
    
    def disable(self) -> bool:
        """Disable motor"""
        # Disabling motor is sending enable command with value 0
        try:
            cmd_id = CAN_BASE_CMD_ID + self.id
            data = bytearray([0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
            
            msg = can.Message(
                arbitration_id=cmd_id,
                data=data,
                is_extended_id=False
            )
            self.controller.can_bus.send(msg)
            return True
        except Exception as e:
            print(f"Error disabling motor: {e}")
            return False
    
    def save_parameters(self) -> bool:
        """Save parameters to non-volatile memory"""
        return self.controller._send_write_command(self.id, MotorCommands.SAVE_PARAMETERS)
    
    def release_brake(self) -> bool:
        """Release brake"""
        return self.controller._send_write_command(self.id, MotorCommands.RELEASE_BRAKE)
    
    def set_position_mode(self) -> bool:
        """Set position control mode"""
        return self.controller.set_parameter(self.id, MotorParams.CONTROL_MODE, ControlModes.POSITION)
    
    def set_speed_mode(self) -> bool:
        """Set speed control mode"""
        return self.controller.set_parameter(self.id, MotorParams.CONTROL_MODE, ControlModes.SPEED)
    
    def set_torque_mode(self) -> bool:
        """Set torque control mode"""
        return self.controller.set_parameter(self.id, MotorParams.CONTROL_MODE, ControlModes.TORQUE)
    
    def set_absolute_position_mode(self) -> bool:
        """Set absolute position mode"""
        return self.controller.set_parameter(self.id, MotorParams.MOTION_MODE, MotionModes.ABSOLUTE)
    
    def set_relative_position_mode(self) -> bool:
        """Set relative position mode"""
        return self.controller.set_parameter(self.id, MotorParams.MOTION_MODE, MotionModes.RELATIVE)
    
    def set_acceleration(self, accel: int) -> bool:
        """Set acceleration"""
        return self.controller.set_parameter(self.id, MotorParams.ACCELERATION, accel)
    
    def set_deceleration(self, decel: int) -> bool:
        """Set deceleration"""
        return self.controller.set_parameter(self.id, MotorParams.DECELERATION, decel)
    
    def set_target_speed(self, speed: int) -> bool:
        """Set target speed"""
        return self.controller.set_parameter(self.id, MotorParams.TARGET_SPEED, speed)
    
    def set_target_position(self, position: int) -> bool:
        """Set target position"""
        return self.controller.set_parameter(self.id, MotorParams.TARGET_POSITION, position)
    
    def set_relative_position(self, position: int) -> bool:
        """Set relative position amount"""
        return self.controller.set_parameter(self.id, MotorParams.RELATIVE_POSITION, position)
    
    def start_motion(self) -> bool:
        """Start motion"""
        return self.controller._send_write_command(self.id, MotorCommands.START_MOTION)
    
    def stop_motion(self) -> bool:
        """Stop motion"""
        return self.controller._send_write_command(self.id, MotorCommands.STOP_MOTION)
    
    # -------------------------------------------------------------------------
    # Parameter Reading Methods
    # -------------------------------------------------------------------------
    
    def read_position(self) -> bool:
        """Read current position"""
        return self.controller.read_parameter(self.id, MotorParams.ACTUAL_POSITION)
    
    def read_speed(self) -> bool:
        """Read current speed"""
        return self.controller.read_parameter(self.id, MotorParams.ACTUAL_SPEED)
    
    def read_current(self) -> bool:
        """Read current motor current"""
        return self.controller.read_parameter(self.id, MotorParams.MOTOR_CURRENT)
    
    def read_status(self) -> bool:
        """Read running status"""
        return self.controller.read_parameter(self.id, MotorParams.RUN_STATUS)
    
    def read_error_code(self) -> bool:
        """Read error code"""
        return self.controller.read_parameter(self.id, MotorParams.ERROR_CODE)
    
    def read_temperature(self) -> bool:
        """Read power module temperature"""
        return self.controller.read_parameter(self.id, MotorParams.POWER_MODULE_TEMP)
    
    def read_all_status(self) -> Dict:
        """Read all status parameters"""
        try:
            # Step 8: Read current motor current
            self.read_current()
            time.sleep(0.1)
            
            # Step 9: Read actual speed
            self.read_speed()
            time.sleep(0.1)
            
            # Step 10: Read current position
            self.read_position()
            time.sleep(0.1)
            
            return self.status
        except Exception as e:
            print(f"Error reading status: {e}")
            return {}
    
    # -------------------------------------------------------------------------
    # Status Getter Methods
    # -------------------------------------------------------------------------
    
    def get_position(self) -> Optional[int]:
        """Get current position value"""
        status = self.controller.get_motor_status(self.id)
        return status.get('position')
    
    def get_speed(self) -> Optional[int]:
        """Get current speed value"""
        status = self.controller.get_motor_status(self.id)
        return status.get('speed')
    
    def get_current(self) -> Optional[int]:
        """Get current motor current value"""
        status = self.controller.get_motor_status(self.id)
        return status.get('motor_current')
    
    def get_status(self) -> Optional[int]:
        """Get running status value"""
        status = self.controller.get_motor_status(self.id)
        return status.get('run_status')
    
    def get_error_code(self) -> Optional[int]:
        """Get error code"""
        status = self.controller.get_motor_status(self.id)
        return status.get('error_code')
    
    def get_temperature(self) -> Optional[int]:
        """Get power module temperature"""
        status = self.controller.get_motor_status(self.id)
        return status.get('power_module_temp')
    
    def get_all_status(self) -> Dict:
        """Get all status parameters"""
        return self.controller.get_motor_status(self.id)
    
    # -------------------------------------------------------------------------
    # Advanced Control Methods
    # -------------------------------------------------------------------------
    
    def move_to_position(self, position: int, timeout: float = 10.0) -> bool:
        """
        Move to specified position and wait for completion
        
        Parameters:
            position: Target position
            timeout: Timeout duration (seconds)
            
        Returns:
            Whether successfully reached target position
        """
        # Set target position
        self.set_target_position(position)
        time.sleep(0.1)
        
        # Start motion
        self.start_motion()
        time.sleep(0.1)
        
        # Wait for reaching target position
        start_time = time.time()
        position_reached = False
        
        while not position_reached and (time.time() - start_time) < timeout:
            # Read current position
            self.read_position()
            time.sleep(0.5)
            
            # Get current position
            current_position = self.get_position()
            
            # Check if target position is reached
            if current_position is not None:
                position_error = abs(current_position - position)
                if position_error < 100:  # Allow 100 counts error
                    position_reached = True
                    print(f"Motor {self.id} reached target position! Current position: {current_position}, Target position: {position}")
                else:
                    print(f"Motor {self.id} moving... Current position: {current_position}, Target position: {position}, Error: {position_error}")
            
            time.sleep(0.5)
        
        if not position_reached:
            print(f"Warning: Motor {self.id} did not reach target position within specified time")
            return False
        
        return True
    
    def setup_position_mode(self) -> bool:
        """
        Set position control mode and configure related parameters
        
        Returns:
            Whether setup was successful
        """
        try:
            # Step 1: Set control mode to position control
            if not self.set_position_mode():
                print(f"Failed to set position control mode: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            # Step 2: Set motion mode to absolute position mode
            if not self.set_absolute_position_mode():
                print(f"Failed to set absolute position mode: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            # Step 3: Set acceleration
            if not self.set_acceleration(100000):
                print(f"Failed to set acceleration: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            # Step 4: Set deceleration
            if not self.set_deceleration(100000):
                print(f"Failed to set deceleration: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            # Step 5: Set target speed
            if not self.set_target_speed(50000):
                print(f"Failed to set target speed: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            print(f"Motor {self.id} position control mode setup successful")
            return True
        except Exception as e:
            print(f"Error setting up position control mode: {e}")
            return False

    def setup_speed_mode(self) -> bool:
        """
        Set speed control mode and configure related parameters
        
        Returns:
            Whether setup was successful
        """
        try:
            # Step 1: Set control mode to speed control
            if not self.set_speed_mode():
                print(f"Failed to set speed control mode: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            # Step 2: Set acceleration
            if not self.set_acceleration(100000):
                print(f"Failed to set acceleration: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            # Step 3: Set deceleration
            if not self.set_deceleration(100000):
                print(f"Failed to set deceleration: Motor {self.id}")
                return False
            time.sleep(0.1)
            
            print(f"Motor {self.id} speed control mode setup successful")
            return True
        except Exception as e:
            print(f"Error setting up speed control mode: {e}")
            return False

    def set_speed_direct(self, speed: int) -> bool:
        """
        Directly set speed value
        
        Parameters:
            speed: Target speed (count/s)
            
        Returns:
            bool: Returns True if successful, False otherwise
        """
        try:
            # Set analog value (target speed) (01 FE 00 00 xx xx)
            return self.controller.set_parameter(self.id, 0xFE, speed)
        except Exception as e:
            print(f"Error setting speed: {e}")
            return False

    def is_enabled(self) -> bool:
        """Check if motor is enabled"""
        try:
            # Read motor enable status
            self.controller.read_parameter(self.id, MotorParams.MOTOR_ENABLE)
            time.sleep(0.05)
            
            # Get motor status
            status = self.controller.get_motor_status(self.id)
            enable_status = status.get('motor_enable', 0)
            
            return enable_status == 1
        except Exception as e:
            print(f"Error checking motor enable status: {e}")
            return False

    def setup_speed_mode_complete(self, target_speed: int = 10000) -> bool:
        """Complete function for setting up speed mode"""
        success = True
        
        # Steps 1-7 same as before...
        
        # 8. Important: Start motion command
        success &= self.start_motion()
        time.sleep(0.1)
        
        return success


# =============================================================================
# 电机控制器类
# =============================================================================

class MotorController:
    """Motor controller class for controlling motors via CAN bus"""
    
    def __init__(self, can_channel: str = 'can0', bitrate: int = 1000000):
        """
        Initialize motor controller
        
        Parameters:
            can_channel: CAN channel name
            bitrate: CAN bus bitrate
        """
        self.motors = {}  # Dictionary to store motor information
        self.motor_objects = {}  # Dictionary to store motor objects
        self.running = False
        self.can_bus = None
        self.can_channel = can_channel
        self.bitrate = bitrate
        self.status_thread = None
        self.status_callback = None
        self.discovered_motors = []  # Store discovered motor IDs from scanning
        self.last_commands = {}  # Store recent commands sent to each motor
    
    # -------------------------------------------------------------------------
    # Connection and Initialization Methods
    # -------------------------------------------------------------------------
    
    def connect(self) -> bool:
        """
        Connect to CAN bus
        
        Returns:
            Whether connection was successful
        """
        try:
            self.can_bus = can.interface.Bus(
                channel=self.can_channel,
                bustype='socketcan',
                bitrate=self.bitrate
            )
            print(f"Connected to CAN bus: {self.can_channel}")
            return True
        except Exception as e:
            print(f"CAN bus connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.can_bus:
            self.stop_status_listener()
            self.can_bus.shutdown()
            self.can_bus = None
            print("Disconnected from CAN bus")
    
    def add_motor(self, motor_id: int, name: str = None) -> CANMotor:
        """
        Add motor to control list
        
        Parameters:
            motor_id: Motor ID (1-8)
            name: Motor name (optional)
            
        Returns:
            Motor object
        """
        if motor_id < 1 or motor_id > 8:
            raise ValueError("Motor ID must be between 1-8")
        
        self.motors[motor_id] = {
            'name': name or f"Motor_{motor_id}",
            'position': 0,
            'speed': 0,
            'motor_current': 0,
            'u_phase_current': 0,
            'v_phase_current': 0,
            'u_phase_voltage': 0,
            'v_phase_voltage': 0,
            'w_phase_voltage': 0,
            'error_code': 0,
            'run_status': 0,
            'digital_input': 0,
            'analog_input': 0,
            'dc_bus_voltage': 0,
            'power_module_temp': 0,
            'continuous_current': 0,
            'peak_current': 0,
            'peak_current_time': 0,
            'can_id': motor_id,
            'online': False,
            'last_update': 0
        }
        
        # Create motor object
        motor = CANMotor(self, motor_id, name)
        self.motor_objects[motor_id] = motor
        
        print(f"Motor added: ID={motor_id}, Name={name or f'Motor_{motor_id}'}")
        return motor
    
    # -------------------------------------------------------------------------
    # Communication Methods
    # -------------------------------------------------------------------------
    
    def _send_write_command(self, motor_id: int, command_index: int) -> bool:
        """Send write command"""
        try:
            # Construct command
            cmd_id = CAN_BASE_CMD_ID + motor_id
            
            # Determine command format based on command index
            if command_index == MotorCommands.START_MOTION:
                data = bytearray([0x00, 0x83])
            elif command_index == MotorCommands.STOP_MOTION:
                data = bytearray([0x00, 0x84])
            elif command_index == MotorCommands.SAVE_PARAMETERS:
                data = bytearray([0x00, 0xE8])
            elif command_index == MotorCommands.RELEASE_BRAKE:
                data = bytearray([0x01, 0x4F, 0x00, 0x00, 0x00, 0x01])
            else:
                # Default command format
                data = bytearray([0x00, command_index])
            
            # Send command - removed detailed printing
            msg = can.Message(
                arbitration_id=cmd_id,
                data=data,
                is_extended_id=False
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            print(f"Error sending write command: {e}")
            return False
    
    def read_parameter(self, motor_id: int, param_index: int) -> bool:
        """Read parameter"""
        try:
            # Construct command
            cmd_id = CAN_BASE_CMD_ID + motor_id
            
            # Determine command format based on parameter index
            if param_index == MotorParams.ERROR_CODE:
                # According to the table, error code should be index 0x1F (31)
                data = bytearray([0x00, 0x1F])
            elif param_index == MotorParams.ACTUAL_SPEED:
                # According to the table, actual speed is index 0x05 (5)
                data = bytearray([0x00, 0x05, 0x00, 0x01])
            elif param_index == MotorParams.ACTUAL_POSITION:
                # According to the table, actual position is index 0x02 (2)
                data = bytearray([0x00, 0x02])
            elif param_index == MotorParams.MOTOR_CURRENT:
                # According to the table, actual motor current is index 0x08 (8)
                data = bytearray([0x00, 0x08])
            elif param_index == MotorParams.RUN_STATUS:
                # According to the table, running status is index 0x20 (32)
                data = bytearray([0x00, 0x20])
            else:
                # Default read command format, also use four-byte format
                data = bytearray([0x00, param_index, 0x00, 0x01])
            
            # Send command - removed detailed printing
            msg = can.Message(
                arbitration_id=cmd_id,
                data=data,
                is_extended_id=False
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            print(f"Error reading parameter: {e}")
            return False
    
    def set_parameter(self, motor_id: int, param_index: int, value: int) -> bool:
        """Set parameter"""
        try:
            # Construct command
            cmd_id = CAN_BASE_CMD_ID + motor_id
            
            # Determine command format based on parameter index
            if param_index == MotorParams.CONTROL_MODE:
                # Control mode: 00 4E 00 00 00 xx
                data = bytearray([0x00, 0x4E, 0x00, 0x00, 0x00, value & 0xFF])
            elif param_index == MotorParams.MOTION_MODE:
                # Motion mode: 00 8D 00 00 00 xx
                data = bytearray([0x00, 0x8D, 0x00, 0x00, 0x00, value & 0xFF])
            elif param_index == MotorParams.ACCELERATION:
                # Acceleration: 00 88 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x88])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.DECELERATION:
                # Deceleration: 00 89 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x89])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.TARGET_SPEED:
                # Target speed: 00 8A xx xx xx xx (little endian)
                data = bytearray([0x00, 0x8A])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.TARGET_POSITION:
                # Target position: 00 86 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x86])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.RELATIVE_POSITION:
                # Relative position: 00 87 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x87])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.MAX_SPEED:
                # Maximum speed: 00 50 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x50])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.MIN_POSITION:
                # Minimum position: 00 52 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x52])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.MAX_POSITION:
                # Maximum position: 00 53 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x53])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.MAX_POSITION_ERROR:
                # Maximum position error: 00 54 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x54])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.MAX_SPEED_ERROR:
                # Maximum speed error: 00 55 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x55])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.STALL_CURRENT:
                # Stall current: 00 56 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x56])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.STALL_SPEED:
                # Stall speed: 00 57 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x57])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.STALL_TIME:
                # Stall time: 00 58 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x58])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.MAX_PHASE_CURRENT:
                # Maximum phase current: 00 62 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x62])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.MAX_SHUTDOWN_CURRENT:
                # Maximum shutdown current: 00 63 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x63])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.POSITION_LOOP_GAIN:
                # Position loop gain: 00 64 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x64])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.SPEED_LOOP_GAIN:
                # Speed loop gain: 00 66 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x66])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            elif param_index == MotorParams.SPEED_LOOP_INTEGRAL:
                # Speed loop integral: 00 67 xx xx xx xx (little endian)
                data = bytearray([0x00, 0x67])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            else:
                # Default parameter format
                data = bytearray([0x00, param_index])
                data.extend([(value >> i) & 0xFF for i in range(0, 32, 8)])
            
            # Send command - removed detailed printing
            msg = can.Message(
                arbitration_id=cmd_id,
                data=data,
                is_extended_id=False
            )
            self.can_bus.send(msg)
            return True
        except Exception as e:
            print(f"Error setting parameter: {e}")
            return False
    
    # -------------------------------------------------------------------------
    # Status Listener Methods
    # -------------------------------------------------------------------------
    
    def start_status_listener(self, callback=None):
        """
        Start status listener thread
        
        Parameters:
            callback: Status update callback function (optional)
        """
        if self.status_thread and self.status_thread.is_alive():
            print("Status listener thread already running")
            return
        
        self.status_callback = callback
        self.status_thread = threading.Thread(target=self._status_listener)
        self.status_thread.daemon = True
        self.status_thread.start()
        print("Status listener thread started")
    
    def stop_status_listener(self):
        """Stop status listener thread"""
        if self.status_thread and self.status_thread.is_alive():
            self.running = False
            self.status_thread.join(timeout=1.0)
            print("Status listener thread stopped")
    
    def _status_listener(self):
        """Status listener thread"""
        self.running = True
        while self.running:
            try:
                message = self.can_bus.recv(timeout=0.1)
                if message:
                    self._process_message(message)
            except Exception as e:
                print(f"Error in status listener: {e}")
                break
    
    def _process_message(self, message):
        """
        Process received CAN message
        
        Parameters:
            message: CAN message object
        """
        msg_id = message.arbitration_id
        
        # Check if it's a status return message (5C0+)
        if CAN_BASE_STATUS_ID <= msg_id < CAN_BASE_STATUS_ID + 16:
            motor_id = msg_id - CAN_BASE_STATUS_ID
            
            # If motor ID is not in the list, add it automatically
            if motor_id not in self.motors:
                self.add_motor(motor_id)
            
            # Get message length
            msg_len = len(message.data)
            
            # Check if it's a response to a write operation
            if msg_len == 1 and message.data[0] == CAN_STATUS_SUCCESS:
                # Write operation successful - reduced printing
                # print(f"Motor {motor_id} parameter write successful")
                
                # Get the most recently written parameter index
                param_index = self._get_last_param_index(motor_id)
                
                # If there's a callback function, call it
                if self.status_callback:
                    self.status_callback(motor_id, self.motors[motor_id], param_index, None, False, True)
                    
            # Check if it's an error response to a write operation
            elif msg_len == 3 and message.data[2] == CAN_STATUS_ERROR:
                # Write operation failed, get error code
                error_code = (message.data[0] << 8) | message.data[1]
                print(f"Motor {motor_id} parameter write failed, error code: 0x{error_code:04X}")
                
                # If there's a callback function, call it and pass error status
                if self.status_callback:
                    self.status_callback(motor_id, self.motors[motor_id], None, None, True, True, error_code)
                    
            # Check if it's a response to a read operation
            elif msg_len >= 5 and message.data[4] == CAN_STATUS_SUCCESS:
                # Read operation successful, parse data
                data_value = (message.data[0] << 24) | (message.data[1] << 16) | \
                            (message.data[2] << 8) | message.data[3]
                
                # Try to get parameter index from recently sent commands
                param_index = self._get_last_param_index(motor_id)
                
                # Update motor status based on parameter index
                if param_index is not None:
                    self._update_motor_param(motor_id, param_index, data_value)
                
                # Update online status and timestamp
                self.motors[motor_id]['online'] = True
                self.motors[motor_id]['last_update'] = time.time()
                
                # If there's a callback function, call it
                if self.status_callback:
                    self.status_callback(motor_id, self.motors[motor_id], param_index, data_value)
                    
            # Check if it's an error response to a read operation
            elif msg_len >= 5 and message.data[4] == CAN_STATUS_ERROR:
                print(f"Motor {motor_id} returned error status")
                
                # If there's a callback function, call it and pass error status
                if self.status_callback:
                    self.status_callback(motor_id, self.motors[motor_id], None, None, True)
    
    def _update_motor_param(self, motor_id, param_index, value):
        """
        Update motor status based on parameter index
        
        Parameters:
            motor_id: Motor ID
            param_index: Parameter index
            value: Parameter value
        """
        # For position value, need to convert unsigned integer to signed integer
        if param_index == MotorParams.ACTUAL_POSITION:
            # Convert 32-bit unsigned integer to signed integer
            if value > 0x7FFFFFFF:  # If highest bit is 1
                value = value - 0x100000000  # Convert to negative
            self.motors[motor_id]['position'] = value
        elif param_index == MotorParams.ACTUAL_SPEED:
            # Speed could also be a signed value
            if value > 0x7FFFFFFF:
                value = value - 0x100000000
            self.motors[motor_id]['speed'] = value
        elif param_index == MotorParams.MOTOR_CURRENT:
            self.motors[motor_id]['motor_current'] = value
        elif param_index == MotorParams.U_PHASE_CURRENT:
            self.motors[motor_id]['u_phase_current'] = value
        elif param_index == MotorParams.V_PHASE_CURRENT:
            self.motors[motor_id]['v_phase_current'] = value
        elif param_index == MotorParams.U_PHASE_VOLTAGE:
            self.motors[motor_id]['u_phase_voltage'] = value
        elif param_index == MotorParams.V_PHASE_VOLTAGE:
            self.motors[motor_id]['v_phase_voltage'] = value
        elif param_index == MotorParams.W_PHASE_VOLTAGE:
            self.motors[motor_id]['w_phase_voltage'] = value
        elif param_index == MotorParams.ERROR_CODE:
            self.motors[motor_id]['error_code'] = value
        elif param_index == MotorParams.RUN_STATUS:
            self.motors[motor_id]['run_status'] = value
        elif param_index == MotorParams.DIGITAL_INPUT:
            self.motors[motor_id]['digital_input'] = value
        elif param_index == MotorParams.ANALOG_INPUT:
            self.motors[motor_id]['analog_input'] = value
        elif param_index == MotorParams.DC_BUS_VOLTAGE:
            self.motors[motor_id]['dc_bus_voltage'] = value
        elif param_index == MotorParams.POWER_MODULE_TEMP:
            self.motors[motor_id]['power_module_temp'] = value
        elif param_index == MotorParams.CONTINUOUS_CURRENT:
            self.motors[motor_id]['continuous_current'] = value
        elif param_index == MotorParams.PEAK_CURRENT:
            self.motors[motor_id]['peak_current'] = value
        elif param_index == MotorParams.PEAK_CURRENT_TIME:
            self.motors[motor_id]['peak_current_time'] = value
        elif param_index == MotorParams.CAN_ID:
            self.motors[motor_id]['can_id'] = value
    
    def _get_last_param_index(self, motor_id):
        """
        Get the most recently written parameter index to the specified motor
        
        Parameters:
            motor_id: Motor ID
            
        Returns:
            Parameter index, None if not recorded
        """
        if motor_id in self.last_commands:
            # Check if command is expired (more than 1 second old)
            if time.time() - self.last_commands[motor_id]['timestamp'] < 1.0:
                return self.last_commands[motor_id]['param_index']
        return None
    
    # -------------------------------------------------------------------------
    # Scan Methods
    # -------------------------------------------------------------------------
    
    def scan_can_ids(self, start_id: int = 1, end_id: int = 14) -> List[int]:
        """
        Scan for motor IDs on the CAN bus
        
        Parameters:
            start_id: Starting ID
            end_id: Ending ID
            
        Returns:
            List of found motor IDs
        """
        found_ids = []
        print(f"Scanning for motor IDs on CAN bus ({start_id}-{end_id})...")
        
        for motor_id in range(start_id, end_id + 1):
            # Reduced printing
            # print(f"Attempting ID: {motor_id}")
            
            # Try multiple reads to increase success rate
            for attempt in range(3):
                try:
                    # Try to read motor position
                    msg = can.Message(
                        arbitration_id=0x640 + motor_id,
                        data=[0x00, 0x63],  # Read position command
                        is_extended_id=False
                    )
                    self.can_bus.send(msg)
                    
                    # Wait for response
                    response = self.can_bus.recv(timeout=0.2)
                    
                    if response and response.arbitration_id == (0x580 + motor_id):
                        print(f"Found motor ID: {motor_id}")
                        found_ids.append(motor_id)
                        break  # Found, exit retry loop
                except Exception as e:
                    # Reduced printing
                    if attempt == 2:  # Only print on last failed attempt
                        print(f"Scan ID {motor_id} failed: {e}")
                
                time.sleep(0.1)  # Brief delay before retrying
        
        return found_ids
    
    # -------------------------------------------------------------------------
    # Helper Methods
    # -------------------------------------------------------------------------
    
    def get_motor(self, motor_id: int) -> Optional[CANMotor]:
        """
        Get motor object
        
        Parameters:
            motor_id: Motor ID
            
        Returns:
            Motor object, None if not exists
        """
        return self.motor_objects.get(motor_id)
    
    def get_all_motors(self) -> List[CANMotor]:
        """
        Get all motor objects
        
        Returns:
            List of motor objects
        """
        return list(self.motor_objects.values())
    
    def get_motor_status(self, motor_id: int) -> Dict:
        """
        Get motor status
        
        Parameters:
            motor_id: Motor ID
            
        Returns:
            Motor status dictionary
        """
        if motor_id not in self.motors:
            print(f"Error: Motor ID {motor_id} does not exist")
            return {}
        
        return self.motors[motor_id]
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        for motor_id in self.motors:
            try:
                # Stop motion
                self._send_write_command(motor_id, MotorCommands.STOP_MOTION)
                time.sleep(0.1)
                # Disable motor
                self.set_parameter(motor_id, MotorParams.MOTOR_ENABLE, 0)
                print(f"Emergency stopped motor {motor_id}")
            except Exception as e:
                print(f"Failed to emergency stop motor {motor_id}: {e}")

    def _wait_for_response(self, motor_id: int, timeout: float = 0.5) -> Optional[can.Message]:
        """
        Wait for motor response
        
        Parameters:
            motor_id: Motor ID
            timeout: Timeout time (seconds)
            
        Returns:
            can.Message: Response message, None if timeout
        """
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                msg = self.can_bus.recv(timeout=0.1)
                if msg:
                    # Check if it's a response from the specified motor
                    if msg.arbitration_id == CAN_BASE_STATUS_ID + motor_id:
                        # Reduced printing
                        # print(f"Received response from motor {motor_id}: ID=0x{msg.arbitration_id:X}, Data={[hex(b) for b in msg.data]}")
                        return msg
            except Exception as e:
                print(f"Error waiting for response: {e}")
                break
        
        print(f"Motor {motor_id} response timeout")
        return None

    def _process_can_message(self, msg):
        """Process received CAN message"""
        try:
            # Check if it's a status return message (0x580+)
            if 0x580 <= msg.arbitration_id < 0x580 + 16:
                motor_id = msg.arbitration_id - 0x580
                
                # Reduced printing
                # print(f"Received message from motor {motor_id}: {msg}")
                
                # Process message...
                
        except Exception as e:
            print(f"Error processing CAN message: {e}")

    def _send_command(self, arbitration_id, data, timeout=0.5, retries=3):
        """Send CAN command and handle errors"""
        for attempt in range(retries):
            try:
                msg = can.Message(
                    arbitration_id=arbitration_id,
                    data=data,
                    is_extended_id=False
                )
                self.can_bus.send(msg)
                return True
            except Exception as e:
                print(f"Failed to send CAN command (attempt {attempt+1}/{retries}): {e}")
                if "Broken pipe" in str(e):
                    print("Detected broken pipe, attempting to reconnect to CAN bus...")
                    self.disconnect()
                    time.sleep(1.0)
                    self.connect()
                    time.sleep(0.5)
                else:
                    time.sleep(0.2)
        
        print(f"Failed to send CAN command, maximum retries ({retries}) reached")
        return False


# =============================================================================
# Callback Functions
# =============================================================================

def status_callback(motor_id, status, param_index=None, data_value=None, is_error=False, is_write=False, error_code=None):
    """
    Example motor status update callback function
    
    Parameters:
        motor_id: Motor ID
        status: Motor status dictionary
        param_index: Parameter index (optional)
        data_value: Returned data value (optional)
        is_error: Whether it's an error status (optional)
        is_write: Whether it's a write operation (optional)
        error_code: Error code (optional)
    """
    if is_error:
        if error_code is not None:
            print(f"Motor {motor_id} returned error status, error code: 0x{error_code:04X}")
        else:
            print(f"Motor {motor_id} returned error status")
    elif is_write:
        # Reduced printing for write success
        if param_index is not None and param_index in [MotorParams.TARGET_POSITION, MotorParams.TARGET_SPEED]:
            param_name = PARAM_NAMES.get(param_index, f"Unknown parameter(0x{param_index:02X})")
            print(f"Motor {motor_id} parameter write successful: {param_name}")
    elif param_index is not None and data_value is not None:
        # For position and speed, may need to convert to signed value for display
        if param_index in [MotorParams.ACTUAL_POSITION, MotorParams.ACTUAL_SPEED]:  # Position or speed
            if data_value > 0x7FFFFFFF:
                display_value = data_value - 0x100000000
            else:
                display_value = data_value
                
            # Only print updates for important parameters
            if param_index in [MotorParams.ACTUAL_POSITION, MotorParams.ACTUAL_SPEED, 
                              MotorParams.MOTOR_CURRENT, MotorParams.ERROR_CODE]:
                param_name = PARAM_NAMES.get(param_index, f"Unknown parameter(0x{param_index:02X})")
                print(f"Motor {motor_id} parameter update: {param_name}={display_value}")
        else:
            # For other parameters, only print important ones
            if param_index in [MotorParams.ERROR_CODE, MotorParams.RUN_STATUS, 
                              MotorParams.MOTOR_CURRENT, MotorParams.POWER_MODULE_TEMP]:
                param_name = PARAM_NAMES.get(param_index, f"Unknown parameter(0x{param_index:02X})")
                print(f"Motor {motor_id} parameter update: {param_name}={data_value}")
    # Removed general status update printing
    # else:
    #     print(f"Motor {motor_id} status update: Speed={status['speed']}, Position={status['position']}")
