# MoveIt Servo 流式运动控制集成指南

本指南介绍了如何使用 MoveIt Servo 在 Dummy 机械臂上实现高频、低延迟的流式控制（适用于视觉伺服、实时遥控或流畅视教）。

## 1. 架构说明与文件清单

为了绕过传统 MoveGroup 规划带来的高延迟，我们采用了 **MoveIt Servo + Python 硬件桥接** 的架构。

### 修改及新增文件：
1.  **`src/dummy_controller/dummy_controller/dummy_servo_hardware.py` (新增)**
    *   **作用**：硬件驱动桥梁。
    *   **功能**：订阅 `/servo_node/command` (50Hz)，提取目标角度并立即通过 Fibre 协议调用 `move_j`。同时以 50Hz 发布 `/joint_states` 供 RViz 和 Servo 闭环。
2.  **`src/dummy_moveit_config/config/servo_streaming.yaml` (新增)**
    *   **作用**：MoveIt Servo 核心配置文件。
    *   **功能**：配置笛卡尔/关节速度限制、奇异点阈值、碰撞检测参数，并定义输入/输出 Topic。
3.  **`src/dummy_moveit_config/launch/servo_streaming.launch.py` (新增)**
    *   **作用**：一键启动脚本。
    *   **功能**：组合启动 RViz、MoveIt Servo 节点、硬件桥接节点和 Robot State Publisher。
4.  **`src/dummy_controller/setup.py` (修改)**
    *   **作用**：注册新节点入口。
    *   **功能**：添加了 `dummy_servo_hardware` 节点的可执行入口。

## 2. 构建与运行步骤

在工作空间根目录下执行以下步骤：

### 编译与环境刷新
```bash
# 编译受影响的包
colcon build --packages-select dummy_controller dummy_moveit_config
# 刷新环境变量
source install/setup.bash
```

### 启动流式模式
```bash
ros2 launch dummy_moveit_config servo_streaming.launch.py
```
*启动后，RViz 将自动开启，并显示当前机械臂的实时姿态。*

## 3. 下达目标与监控

### 激活 Servo 引擎 (重要！)
出于安全原因，MoveIt Servo 节点启动后默认处于**暂停状态**。每次启动 Launch 文件后，你**必须**手动调用一次服务来激活它：
```bash
ros2 service call /moveit_servo/start_servo std_srvs/srv/Trigger
```
*激活成功后，`ros2 topic echo /servo_node/status` 才会开始跳动（通常输出 0）。*

### 下达移动目标 (流式 Topic)
您需要通过 Python 代码或终端不断发布 `TwistStamped` 消息到 `/servo_node/delta_twist_cmds`。

**终端测试示例（让末端沿 X 轴以 0.05m/s 匀速移动）：**
```bash
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "
{
  header: { stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link' },
  twist: {
    linear: {x: 0.05, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

### 检查运动情况
1.  **RViz 镜像**：RViz 里的 3D 模型会实时镜像真实机械臂的动作。如果 RViz 不动，请检查 `/joint_states` 话题是否有数据输出。
2.  **安全性**：如果机械臂停止移动，请检查终端输出。MoveIt Servo 会在机械臂接近奇异点（Singularity）或即将发生自碰撞时自动刹车。
3.  **状态监控**：通过 `ros2 topic echo /servo_node/status` 查看 Servo 引擎的健康状态（Status=1 表示一切正常）。

## 4. 注意事项
*   **持续性**：Servo 是一种“死人开关”机制，如果你停止发布消息超过 0.1 秒（由 `incoming_command_timeout` 决定），机械臂将立即停止。
*   **下位机频率**：如果下位机执行 `move_j` 有抖动，请检查下位机是否有专门的“透传模式”或关闭内部的加减速曲线规划。
