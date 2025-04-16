from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import json

def generate_launch_description():
    # 声明 launch 参数
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value='can0',
        description='CAN interface channel (e.g., can0, can1)'
    )
    
    scan_motors_arg = DeclareLaunchArgument(
        'scan_motors_on_startup',
        default_value='false',
        description='Whether to scan for motors on startup'
    )
    
    motor_ids_arg = DeclareLaunchArgument(
        'motor_ids',
        default_value='[1, 2, 3, 4]',
        description='Default motor IDs to use if not scanning'
    )
    
    # 新增：位置控制参数
    position_control_arg = DeclareLaunchArgument(
        'position_control',
        default_value='false',
        description='Whether to start position control'
    )
    
    # 新增：位置控制目标参数 (格式: "1:10000,3:20000")
    position_targets_arg = DeclareLaunchArgument(
        'position_targets',
        default_value='1:10000,3:20000',
        description='Position targets for motors as string (format: "id:pos,id:pos")'
    )

    # 创建 CAN Bridge 节点
    can_bridge_node = Node(
        package='erob_can_bridge',
        executable='can_bridge_node',
        name='can_bridge_node',
        output='screen',
        parameters=[{
            'can_channel': LaunchConfiguration('can_channel'),
            'scan_motors_on_startup': LaunchConfiguration('scan_motors_on_startup'),
            'motor_ids': LaunchConfiguration('motor_ids')
        }]
    )
    
    # 创建多电机位置控制节点
    multi_position_node = Node(
        condition=IfCondition(LaunchConfiguration('position_control')),
        package='erob_can_bridge',
        executable='multi_position_control',
        name='multi_position_control_node',
        output='screen',
        parameters=[{
            'position_targets': LaunchConfiguration('position_targets')
        }]
    )

    return LaunchDescription([
        can_channel_arg,
        scan_motors_arg,
        motor_ids_arg,
        position_control_arg,
        position_targets_arg,
        can_bridge_node,
        multi_position_node
    ]) 