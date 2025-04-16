# EROB CAN Bridge

**Version:** 1.0.0  
**Release Date:** 2025-04-16

A ROS 2 package for controlling motors via CAN bus.

This package can be used for upper-level development on Linux systems or for rapid demo development through ROS 2. We will add torque mode functionality and improve motor status diagnostics in future updates. Currently, only position and velocity modes have been tested, which can be used to quickly drive single or multiple eRob devices via CAN.

## Overview

EROB CAN Bridge provides bridging functionality between ROS 2 and CAN bus motor controllers, supporting:
- Motor discovery and management
- Position and velocity control
- Status monitoring
- Emergency stop functionality

## Prerequisites

- ROS 2 (tested on Humble)
- Python 3.8+
- CAN interface hardware
- `can-utils` package for SocketCAN

## Installation

### 1. Install Dependencies

```bash
# Install CAN tools
sudo apt install can-utils

# Install Python dependencies
pip install python-can
```

### 2. Clone Repository

```bash
mkdir -p ~/erob_ws/src
cd ~/erob_ws/src
git clone git@github.com:ZeroErrControl/eRob_CAN_Bridge.git
```

### 3. Build Package

```bash
cd ~/erob_ws
colcon build
source install/setup.bash
```

## Configure SocketCAN

Before using the CAN Bridge, you need to configure the SocketCAN interface:

```bash
# Set up CAN interface
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

# Check CAN interface status
ip -details link show can0

# Monitor CAN bus communication (for debugging)
candump can0
```

## Usage

### Launch CAN Bridge Node

Basic launch command:

```bash
# Launch with default parameters
ros2 launch erob_can_bridge can_bridge.launch.py

# Specify CAN channel and motor IDs
ros2 launch erob_can_bridge can_bridge.launch.py can_channel:=can0 scan_motors_on_startup:=false motor_ids:='[1, 3, 4, 5]'
```

### Motor Control Commands

#### Position Control

##### Using robust_position_control.sh Script

The simplest way is to use the provided `robust_position_control.sh` script:

```bash
# Control motor 1 to move to position 10000
./robust_position_control.sh 1 10000
```

##### Using Broadcast Position Command

You can control the position of all motors using broadcast commands:

```bash
# Set all motors to position 10000
ros2 topic pub --once /broadcast_position_cmd std_msgs/msg/Int32 "data: 10000"

# Set all motors to position -5000
ros2 topic pub --once /broadcast_position_cmd std_msgs/msg/Int32 "data: -5000"

# Return all motors to home position
ros2 topic pub --once /broadcast_position_cmd std_msgs/msg/Int32 "data: 0"
```

Note: Before using broadcast position commands, ensure all motors are initialized in position control mode using `ros2 service call /init_position_mode std_srvs/srv/Trigger "{}"`.

##### Manually Execute Position Control Command Sequence

For more fine-grained control, you can manually execute the following command sequence:

```bash
# 1. Stop all motors
ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"
sleep 2

# 2. Disable all motors
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: false"
sleep 2

# 3. Initialize position control mode
ros2 service call /init_position_mode std_srvs/srv/Trigger "{}"
sleep 2

# 4. Enable all motors
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"
sleep 2

# 5. Send position command to specific motor (ID=1, position=10000)
ros2 topic pub --once /position_cmd std_msgs/msg/Int32MultiArray "data: [1, 10000]"
```

#### Velocity Control

##### Using SetBool Service for Velocity Control

The simplest way is to use the `sync_set_speed` service:

```bash
# Control all motors with forward velocity (10000)
ros2 service call /sync_set_speed std_srvs/srv/SetBool "data: true"

# Control all motors with reverse velocity (-10000)
ros2 service call /sync_set_speed std_srvs/srv/SetBool "data: false"
```

##### Using Broadcast Velocity Command

You can control the velocity of all motors using broadcast commands:

```bash
# Set all motors to velocity 5000
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 5000"

# Set all motors to velocity -5000 (reverse)
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: -5000"

# Stop all motors
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 0"
```

##### Using Simplified Velocity Command for Individual Motors

For motors already initialized in velocity mode, you can use simplified velocity commands:

```bash
# Send simplified velocity command to specific motor (ID=1, velocity=5000)
ros2 topic pub --once /speed_cmd_simple std_msgs/msg/Int32MultiArray "data: [1, 5000]"

# Send simplified velocity command to specific motor (ID=3, velocity=-5000)
ros2 topic pub --once /speed_cmd_simple std_msgs/msg/Int32MultiArray "data: [3, -5000]"
```

##### Manually Execute Velocity Control Command Sequence

For more fine-grained control, you can manually execute the following command sequence:

```bash
# 1. Stop all motors
ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"
sleep 2

# 2. Disable all motors
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: false"
sleep 2

# 3. Initialize velocity control mode
ros2 service call /init_speed_mode std_srvs/srv/Trigger "{}"
sleep 2

# 4. Enable all motors
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"
sleep 2

# 5. Send velocity command to specific motor (ID=1, velocity=5000)
ros2 topic pub --once /speed_cmd std_msgs/msg/Int32MultiArray "data: [1, 5000]"
```

### Synchronous Control of Multiple Motors

#### Synchronously Start All Motors

```bash
# Synchronously start all configured motors
ros2 service call /sync_start_motors std_srvs/srv/Trigger "{}"
```

### Motor Enable Control

```bash
# Enable all motors
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"

# Disable all motors
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: false"
```

### Emergency Stop

```bash
# Emergency stop all motors
ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"
```

### Error Handling

```bash
# Read error codes
ros2 service call /read_error_code std_srvs/srv/Trigger "{}"

# Clear errors
ros2 service call /clear_error std_srvs/srv/Trigger "{}"
```

### Monitor Motor Status

```bash
# Monitor motor status
ros2 topic echo /motor_status

# Monitor motor position
ros2 topic echo /motor_position

# Monitor motor velocity
ros2 topic echo /motor_speed

# Monitor motor current
ros2 topic echo /motor_current

# Monitor motor errors
ros2 topic echo /motor_error
```

## Troubleshooting

### CAN Interface Not Found

- Check if interface is up: `ip link show can0`
- Try restarting the interface: `sudo ip link set down can0 && sudo ip link set up can0`

### Motors Not Detected

- Confirm motors are powered
- Check CAN bus connections
- Use `candump can0` to verify CAN communication
- Try manually specifying motor IDs: `motor_ids:="[1, 2, 3, 4, 5]"`

### Velocity Control Not Working

- Ensure motors are properly initialized for velocity control: `ros2 service call /init_speed_mode std_srvs/srv/Trigger "{}"`
- Ensure motors are enabled: `ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"`
- Check for error states: `ros2 service call /read_error_code std_srvs/srv/Trigger "{}"`
- Try using broadcast commands: `ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 5000"`
- Ensure you send start command after setting new velocity: `ros2 service call /sync_start_motors std_srvs/srv/Trigger "{}"`

### Position Control Not Working

- Ensure motors are properly initialized for position control
- Ensure motors are enabled
- Check for error states
- Try using direct control scripts, bypassing the ROS 2 interface

### Permission Issues

- Ensure your user has permission to access the CAN interface:
  ```bash
  sudo usermod -a -G dialout $USER
  # Log out and log back in for changes to take effect
  ```

## Advanced Usage

### Combined Commands (One-line Operations)

```bash
# Launch node -> Initialize velocity mode -> Enable motors -> Set velocity
ros2 launch erob_can_bridge can_bridge.launch.py can_channel:=can0 scan_motors_on_startup:=false motor_ids:='[1, 2, 3, 4, 5]' && \
sleep 2 && \
ros2 service call /init_speed_mode std_srvs/srv/Trigger "{}" && \
sleep 1 && \
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true" && \
sleep 1 && \
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 5000"
```

### Custom Control Scripts

You can create your own control scripts to automate motor control processes according to specific requirements. Refer to `motor_control.sh` as an example.

## Configuration Parameters

The node can be configured with the following parameters:

- `can_channel` (string, default: 'can0'): CAN interface channel
- `can_bitrate` (int, default: 1000000): CAN bus bitrate (bits/second)
- `scan_motors_on_startup` (bool, default: true): Whether to scan for motors at startup
- `motor_ids` (int[], default: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]): Default motor IDs if not scanning

## Available Topics

### Publishers
- `/motor_status` (std_msgs/String): Motor status in JSON format
- `/motor_position` (std_msgs/Int32): Current motor position
- `/motor_speed` (std_msgs/Int32): Current motor velocity
- `/motor_current` (std_msgs/Float32): Current motor current (amperes)
- `/motor_error` (std_msgs/String): Error information in JSON format

### Subscribers
- `/position_cmd` (std_msgs/Int32MultiArray): Position command [motor_id, position]
- `/speed_cmd` (std_msgs/Int32MultiArray): Velocity command [motor_id, velocity]
- `/speed_cmd_simple` (std_msgs/Int32MultiArray): Simplified velocity command [motor_id, velocity]
- `/motor_enable` (std_msgs/Bool): Enable/disable all motors
- `/broadcast_position_cmd` (std_msgs/Int32): Position command for all motors
- `/broadcast_speed_cmd` (std_msgs/Int32): Velocity command for all motors

## Available Services
- `/scan_motors` (std_srvs/Trigger): Scan for motors on the CAN bus
- `/emergency_stop` (std_srvs/Trigger): Emergency stop all motors
- `/set_motor_enable` (std_srvs/SetBool): Enable/disable all motors
- `/init_position_mode` (std_srvs/Trigger): Initialize all motors for position control mode
- `/init_speed_mode` (std_srvs/Trigger): Initialize all motors for velocity control mode
- `/read_error_code` (std_srvs/Trigger): Read error codes from all motors
- `/clear_error` (std_srvs/Trigger): Clear errors on all motors
- `/sync_start_motors` (std_srvs/Trigger): Synchronously start all motors
- `/sync_set_speed` (std_srvs/SetBool): Synchronously set velocity for all motors (true=forward, false=reverse)

## Contributing

Issues and pull requests are welcome!

## License

[License Information]