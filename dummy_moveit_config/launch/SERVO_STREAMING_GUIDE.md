--moveit实现流式控制

--新增文件
  1. src/dummy_controller/dummy_controller/dummy_servo_hardware.py
  作用：作为硬件的直接驱动桥梁。它抛弃了缓慢的 Action
  2. src/dummy_moveit_config/config/servo_streaming.yaml
  作用：MoveIt Servo 的核心计算配置文件。
  3. src/dummy_moveit_config/launch/servo_streaming.launch.py
  作用：流式模式的专属一键启动脚本。它抛弃了 ros2_control_node，直接将 Servo 引擎与你的 Python
--修改文件
  1. src/dummy_controller/setup.py
  修改原因：将你新写的 Python 硬件节点注册到 ROS 2 系统中，使其可以通过 ros2 run 或 launch 启动。
  2. src/dummy_controller/SERVO_STREAMING_GUIDE.md
  修改原因：记录最佳实践，特别是我们后来发现的必须手动激活 Servo 引擎的“坑”。


--gemini目录改动程序后编译
cd ~/apps/dummy_ws
(恢复备份)cp -r ~/apps/backup/dummy_ws_20260328/src/* ~/apps/gemini/dummy_ws/src/
cp -r ~/apps/gemini/dummy_ws/src/* ~/apps/dummy_ws/src/
colcon build
source install/setup.bash

--启动机械臂
ros2 launch dummy_moveit_config servo_streaming.launch.py
--启动后触发响应
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
--检查发送目标是否已经触发响应
ros2 topic echo /servo_node/status
--启动机械臂后指定偏移以下，避免Radians: [0. 0. 0. 0. 0. 0.]导致逆解遇到奇点
ros2 topic pub --once /servo_node/delta_joint_cmds control_msgs/msg/JointJog "{header: {stamp: 'now', frame_id: 'base_link'}, joint_names: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5','Joint6'], velocities: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}"

--水平向前
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
--水平向后
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
--水平向左
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
--水平向右
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: -0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
--垂直向上
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: 0.05}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
--垂直向下
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: -0.05}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

--前倾
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: -0.0}, angular: {x: 0.1, y: 0.0, z: 0.0}}}"
--后仰
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: -0.0}, angular: {x: -0.1, y: 0.0, z: 0.0}}}"
--旋左
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: -0.0}, angular: {x: 0.0, y: 0.1, z: 0.0}}}"
--旋右
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: -0.0}, angular: {x: 0.0, y: -0.1, z: 0.0}}}"
--摇左
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: -0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"
--摇右
ros2 topic pub --rate 50 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.0, z: -0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}}"



