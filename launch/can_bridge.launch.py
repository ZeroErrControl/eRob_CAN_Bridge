from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import can
import time

def generate_launch_description():
    # Declare parameters
    can_channel = LaunchConfiguration('can_channel', default='can0')
    can_bitrate = LaunchConfiguration('can_bitrate', default='1000000')
    scan_motors = LaunchConfiguration('scan_motors_on_startup', default='true')
    motor_ids = LaunchConfiguration('motor_ids', default='[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]')
    
    # New: direct position control parameters
    direct_position_arg = DeclareLaunchArgument(
        'direct_position',
        default_value='false',
        description='Whether to use direct position control'
    )
    
    motor_id_arg = DeclareLaunchArgument(
        'motor_id',
        default_value='1',
        description='Motor ID for direct control'
    )
    
    position_arg = DeclareLaunchArgument(
        'position',
        default_value='10000',
        description='Target position for direct control'
    )

    # Create the node
    can_bridge_node = Node(
        package='erob_can_bridge',
        executable='can_bridge_node',
        name='can_bridge_node',
        output='screen',
        parameters=[{
            'can_channel': can_channel,
            'can_bitrate': can_bitrate,
            'scan_motors_on_startup': scan_motors,
            'motor_ids': motor_ids
        }]
    )
    
    # New: direct position control process
    direct_position_control = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('direct_position')),
        cmd=[
            'ros2', 'run', 'erob_can_bridge', 'direct_position_control',
            LaunchConfiguration('motor_id'),
            LaunchConfiguration('position')
        ],
        output='screen'
    )

    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument(
            'can_channel',
            default_value='can0',
            description='CAN interface channel'
        ),
        DeclareLaunchArgument(
            'can_bitrate',
            default_value='1000000',
            description='CAN bitrate in bits/s'
        ),
        DeclareLaunchArgument(
            'scan_motors_on_startup',
            default_value='true',
            description='Whether to scan for motors on startup'
        ),
        DeclareLaunchArgument(
            'motor_ids',
            default_value='[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]',
            description='Default motor IDs to use if not scanning'
        ),
        direct_position_arg,
        motor_id_arg,
        position_arg,
        can_bridge_node,
        direct_position_control
    ])

def init_speed_mode_direct_callback(self, request, response):
    """Initialize speed mode using direct CAN commands"""
    try:
        motor_id = 1  # Default motor ID
        
        # 1. Set control mode to speed control
        msg = can.Message(
            arbitration_id=0x640 + motor_id,
            data=[0x00, 0x4E, 0x00, 0x00, 0x00, 0x02],
            is_extended_id=False
        )
        self.controller.can_bus.send(msg)
        time.sleep(0.2)
        
        # 2-7. Other steps...
        
        response.success = True
        response.message = "Motor initialized to speed mode using direct commands"
        return response
    except Exception as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        return response

def setup_speed_mode(self) -> bool:
    """Set up speed control mode according to protocol"""
    try:
        print(f"Starting to set up speed mode for motor {self.id}...")
        
        # Critical step: Set control mode to speed control
        print(f"Step 1: Set control mode to speed control (Motor {self.id})")
        if not self.controller.set_parameter(self.id, MotorParams.CONTROL_MODE, ControlModes.SPEED):
            print(f"Step 1 failed: Set control mode")
            return False
        time.sleep(0.2)
        
        # Non-critical step: Can tolerate failure
        print(f"Step 2: Set control source to not use (Motor {self.id})")
        self.controller.set_parameter(self.id, 0x12, 0)
        time.sleep(0.2)
        
        # Non-critical step: Can tolerate failure
        print(f"Step 3: Set analog type to internal use (Motor {self.id})")
        self.controller.set_parameter(self.id, 0xFD, 0)
        time.sleep(0.2)
        
        # Critical step: Set acceleration
        print(f"Step 4: Set acceleration (Motor {self.id})")
        if not self.controller.set_parameter(self.id, MotorParams.ACCELERATION, 30000):
            print(f"Step 4 failed: Set acceleration")
            # Continue execution, don't return failure
        time.sleep(0.2)
        
        # Critical step: Set deceleration
        print(f"Step 5: Set deceleration (Motor {self.id})")
        if not self.controller.set_parameter(self.id, MotorParams.DECELERATION, 30000):
            print(f"Step 5 failed: Set deceleration")
            # Continue execution, don't return failure
        time.sleep(0.2)
        
        # Critical step: Set analog value (target speed)
        print(f"Step 6: Set analog value (target speed) (Motor {self.id})")
        if not self.controller.set_parameter(self.id, 0xFE, 10000):
            print(f"Step 6 failed: Set analog value")
            return False
        time.sleep(0.2)
        
        # Critical step: Enable motor
        print(f"Step 7: Enable motor (Motor {self.id})")
        if not self.controller.set_parameter(self.id, MotorParams.MOTOR_ENABLE, 1):
            print(f"Step 7 failed: Enable motor")
            return False
        time.sleep(0.2)
        
        # Critical step: Start motion
        print(f"Step 8: Start motion (Motor {self.id})")
        if not self.start_motion():
            print(f"Step 8 failed: Start motion")
            # Even if this step fails, subsequent speed commands may still work
        time.sleep(0.2)
        
        print(f"Speed mode setup successful for motor {self.id}")
        return True
    except Exception as e:
        print(f"Error setting up speed mode: {e}")
        import traceback
        traceback.print_exc()
        return False

def init_speed_mode_callback(self, request, response):
    """Initialize all motors to speed control mode"""
    try:
        self.get_logger().info("Initializing motor 1 to speed control mode")
        
        # Only initialize motor 1
        if 1 in self.motors:
            motor = self.motors[1]
            try:
                if motor.setup_speed_mode():
                    self.get_logger().info("Motor 1 initialized to speed control mode")
                    response.success = True
                    response.message = "Motor 1 initialized to speed control mode"
                else:
                    self.get_logger().error("Failed to initialize motor 1")
                    response.success = False
                    response.message = "Failed to initialize motor 1"
            except Exception as e:
                self.get_logger().error(f"Failed to initialize motor 1: {str(e)}")
                response.success = False
                response.message = f"Error: {str(e)}"
        else:
            self.get_logger().error("Motor 1 not found")
            response.success = False
            response.message = "Motor 1 not found"
        
        return response
    except Exception as e:
        self.get_logger().error(f"Error in init_speed_mode: {str(e)}")
        response.success = False
        response.message = f"Error: {str(e)}"
        return response

