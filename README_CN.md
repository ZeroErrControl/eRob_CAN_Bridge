# EROB CAN Bridge

**版本:** 1.0.0  
**发布日期:** 2025-04-16

一个用于通过 CAN 总线控制电机的 ROS 2 包。

该包可用于 Linux 系统下的上位机开发或通过 ROS 2 快速开发演示。我们后续会加入力矩模式的功能，完善电机状态诊断功能。目前只测试了位置模式和速度模式的功能，用于通过 CAN 快速驱动单台或多台 eRob 设备。

## 概述

EROB CAN Bridge 提供了 ROS 2 与 CAN 总线电机控制器之间的桥接功能，支持：
- 电机发现和管理
- 位置和速度控制
- 状态监控
- 紧急停止功能

## 前提条件

- ROS 2 (已在 Humble 上测试)
- Python 3.8+
- CAN 接口硬件
- `can-utils` 用于 SocketCAN

## 安装

### 1. 安装依赖

```bash
# 安装 CAN 工具
sudo apt install can-utils

# 安装 Python 依赖
pip install python-can
```

### 2. 克隆仓库

```bash
mkdir -p ~/erob_ws/src
cd ~/erob_ws/src
git clone git@github.com:ZeroErrControl/eRob_CAN_Bridge.git
```

### 3. 构建包

```bash
cd ~/erob_ws
colcon build
source install/setup.bash
```

## 配置 SocketCAN

在使用 CAN Bridge 之前，需要先配置 SocketCAN 接口：

```bash
# 设置 CAN 接口
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

# 查看 CAN 接口状态
ip -details link show can0

# 监控 CAN 总线通信（调试用）
candump can0
```

## 使用方法

### 启动 CAN Bridge 节点

基本启动命令：

```bash
# 使用默认参数启动
ros2 launch erob_can_bridge can_bridge.launch.py

# 指定 CAN 通道和电机 ID
ros2 launch erob_can_bridge can_bridge.launch.py can_channel:=can0 scan_motors_on_startup:=false motor_ids:='[1, 3, 4, 5]'
```

### 电机控制命令

#### 位置控制

##### 使用 robust_position_control.sh 脚本

最简单的方法是使用提供的 `robust_position_control.sh` 脚本：

```bash
# 控制电机1移动到位置10000
./robust_position_control.sh 1 10000
```

##### 使用广播位置命令

注意：在使用广播位置命令之前，请确保所有电机都已初始化为位置控制模式，使用命令 
`ros2 service call /init_position_mode std_srvs/srv/Trigger "{}"`。

可以通过广播命令控制所有电机的位置：

```bash
# 设置所有电机位置为10000
ros2 topic pub --once /broadcast_position_cmd std_msgs/msg/Int32 "data: 10000"

# 设置所有电机位置为-5000
ros2 topic pub --once /broadcast_position_cmd std_msgs/msg/Int32 "data: -5000"

# 将所有电机返回到原点位置
ros2 topic pub --once /broadcast_position_cmd std_msgs/msg/Int32 "data: 0"
```



##### 手动执行位置控制命令序列

如果需要更精细的控制，可以手动执行以下命令序列：

```bash
# 1. 停止所有电机
ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"
sleep 2

# 2. 失能所有电机
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: false"
sleep 2

# 3. 初始化位置控制模式
ros2 service call /init_position_mode std_srvs/srv/Trigger "{}"
sleep 2

# 4. 使能所有电机
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"
sleep 2

# 5. 发送位置命令到特定电机 (ID=1, 位置=10000)
ros2 topic pub --once /position_cmd std_msgs/msg/Int32MultiArray "data: [1, 10000]"
```

#### 速度控制

##### 使用 SetBool 服务控制速度

最简单的方法是使用 `sync_set_speed` 服务：

```bash
# 控制所有电机正向速度 (10000)
ros2 service call /sync_set_speed std_srvs/srv/SetBool "data: true"

# 控制所有电机反向速度 (-10000)
ros2 service call /sync_set_speed std_srvs/srv/SetBool "data: false"
```

##### 使用广播速度命令

可以通过广播命令控制所有电机的速度：

```bash
# 设置所有电机速度为5000
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 5000"

# 设置所有电机速度为-5000（反向）
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: -5000"

# 停止所有电机
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 0"
```

##### 使用简化的速度命令控制单个电机

对于已经初始化为速度模式的电机，可以使用简化的速度命令：

```bash
# 发送简化的速度命令到特定电机 (ID=1, 速度=5000)
ros2 topic pub --once /speed_cmd_simple std_msgs/msg/Int32MultiArray "data: [1, 5000]"

# 发送简化的速度命令到特定电机 (ID=3, 速度=-5000)
ros2 topic pub --once /speed_cmd_simple std_msgs/msg/Int32MultiArray "data: [3, -5000]"
```

##### 手动执行速度控制命令序列

如果需要更精细的控制，可以手动执行以下命令序列：

```bash
# 1. 停止所有电机
ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"
sleep 2

# 2. 失能所有电机
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: false"
sleep 2

# 3. 初始化速度控制模式
ros2 service call /init_speed_mode std_srvs/srv/Trigger "{}"
sleep 2

# 4. 使能所有电机
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"
sleep 2

# 5. 发送速度命令到特定电机 (ID=1, 速度=5000)
ros2 topic pub --once /speed_cmd std_msgs/msg/Int32MultiArray "data: [1, 5000]"
```

### 同步控制多个电机

#### 同步启动所有电机

```bash
# 同步启动所有已配置的电机
ros2 service call /sync_start_motors std_srvs/srv/Trigger "{}"
```

### 电机使能控制

```bash
# 使能所有电机
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"

# 失能所有电机
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: false"
```

### 紧急停止

```bash
# 紧急停止所有电机
ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"
```

### 错误处理

```bash
# 读取错误代码
ros2 service call /read_error_code std_srvs/srv/Trigger "{}"

# 清除错误
ros2 service call /clear_error std_srvs/srv/Trigger "{}"
```

### 监控电机状态

```bash
# 监控电机状态
ros2 topic echo /motor_status

# 监控电机位置
ros2 topic echo /motor_position

# 监控电机速度
ros2 topic echo /motor_speed

# 监控电机电流
ros2 topic echo /motor_current

# 监控电机错误
ros2 topic echo /motor_error
```

## 常见问题排查

### CAN 接口未找到

- 检查接口是否启动：`ip link show can0`
- 尝试重启接口：`sudo ip link set down can0 && sudo ip link set up can0`

### 未检测到电机

- 确认电机已通电
- 检查 CAN 总线连接
- 使用 `candump can0` 验证 CAN 通信
- 尝试手动指定电机 ID：`motor_ids:="[1, 2, 3, 4, 5]"`

### 速度控制不起作用

- 确保已正确初始化为速度控制模式：`ros2 service call /init_speed_mode std_srvs/srv/Trigger "{}"`
- 确保电机已使能：`ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true"`
- 检查是否有错误状态：`ros2 service call /read_error_code std_srvs/srv/Trigger "{}"`
- 尝试使用广播命令：`ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 5000"`
- 确保每次设置新速度后都发送启动命令：`ros2 service call /sync_start_motors std_srvs/srv/Trigger "{}"`

### 位置控制不起作用

- 确保已正确初始化为位置控制模式
- 确保电机已使能
- 检查是否有错误状态
- 尝试使用直接控制脚本，绕过 ROS 2 接口

### 权限问题

- 确保您的用户有权限访问 CAN 接口：
  ```bash
  sudo usermod -a -G dialout $USER
  # 注销并重新登录以使更改生效
  ```

## 高级用法

### 组合命令（一键式操作）

```bash
# 启动节点 -> 初始化速度模式 -> 使能电机 -> 设置速度
ros2 launch erob_can_bridge can_bridge.launch.py can_channel:=can0 scan_motors_on_startup:=false motor_ids:='[1, 2, 3, 4, 5]' && \
sleep 2 && \
ros2 service call /init_speed_mode std_srvs/srv/Trigger "{}" && \
sleep 1 && \
ros2 topic pub --once /motor_enable std_msgs/msg/Bool "data: true" && \
sleep 1 && \
ros2 topic pub --once /broadcast_speed_cmd std_msgs/msg/Int32 "data: 5000"
```

### 自定义控制脚本

您可以创建自己的控制脚本，根据特定需求自动化电机控制流程。参考 `motor_control.sh` 作为示例。

## 配置参数

节点可以通过以下参数进行配置：

- `can_channel` (string, 默认: 'can0'): CAN 接口通道
- `can_bitrate` (int, 默认: 1000000): CAN 总线比特率（比特/秒）
- `scan_motors_on_startup` (bool, 默认: true): 是否在启动时扫描电机
- `motor_ids` (int[], 默认: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]): 如果不扫描，则使用的默认电机 ID

## 可用话题

### 发布者
- `/motor_status` (std_msgs/String): JSON 格式的电机状态
- `/motor_position` (std_msgs/Int32): 当前电机位置
- `/motor_speed` (std_msgs/Int32): 当前电机速度
- `/motor_current` (std_msgs/Float32): 当前电机电流（安培）
- `/motor_error` (std_msgs/String): JSON 格式的错误信息

### 订阅者
- `/position_cmd` (std_msgs/Int32MultiArray): 位置命令 [motor_id, position]
- `/speed_cmd` (std_msgs/Int32MultiArray): 速度命令 [motor_id, speed]
- `/speed_cmd_simple` (std_msgs/Int32MultiArray): 简化的速度命令 [motor_id, speed]
- `/motor_enable` (std_msgs/Bool): 使能/失能所有电机
- `/broadcast_position_cmd` (std_msgs/Int32): 所有电机的位置命令
- `/broadcast_speed_cmd` (std_msgs/Int32): 所有电机的速度命令

## 可用服务
- `/scan_motors` (std_srvs/Trigger): 扫描 CAN 总线上的电机
- `/emergency_stop` (std_srvs/Trigger): 紧急停止所有电机
- `/set_motor_enable` (std_srvs/SetBool): 使能/失能所有电机
- `/init_position_mode` (std_srvs/Trigger): 初始化所有电机为位置控制模式
- `/init_speed_mode` (std_srvs/Trigger): 初始化所有电机为速度控制模式
- `/read_error_code` (std_srvs/Trigger): 读取所有电机的错误代码
- `/clear_error` (std_srvs/Trigger): 清除所有电机的错误
- `/sync_start_motors` (std_srvs/Trigger): 同步启动所有电机
- `/sync_set_speed` (std_srvs/SetBool): 同步设置所有电机的速度 (true=正向, false=反向)

## 贡献

欢迎提交问题和拉取请求！

## 许可证

[许可证信息] 

