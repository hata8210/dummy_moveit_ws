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

--前倾ros2 launch dummy_moveit_config servo_streaming.launch.py
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


--水平向前（发送半秒）
ros2 topic pub --rate 50 -t 25 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"


--增加http流式api服务


增加和改动文件
1. 新增文件：src/dummy_server/server/stream_api_server.py
这是核心服务端程序，采用 FastAPI 框架，将 HTTP 请求转换为 MoveIt Servo 的实时指令。
  2. 新增文件：src/dummy_server/server/stream_api_client_demo.py
  这是一个不依赖 ROS 2 环境的纯 Python 调用示例。
  3. 修改文件：src/dummy_server/CMakeLists.txt
   - 安装配置：在 install(PROGRAMS ...) 列表中添加了 stream_api_server.py，确保在执行 colcon build
运行建议
   1. 启动服务端（运行在 ROS 2 环境）：
   1    # 确保 MoveIt Servo 节点已启动后运行
   2    python3 src/dummy_server/server/stream_api_server.py
   2. 启动客户端 Demo（运行在任意联网电脑）：
   1    python3 src/dummy_server/server/stream_api_client_demo.py



dummy_server中的api_server.py把moveit的plan和execute封装成了可以通过http调用的api，每次调用输入移动目标位姿参数，然后由moveit规划与移动。
但以上方式适合完成一个长距离移动规划，目前我们打算新增加一种流式实时控制方式的stream_api，api将提供给遥控器调用，推拉杆则触发各个方向移动，拉杆数值区间[0.0,1.0]，0为此方向静止，1为全速前进。
目前需要先把给遥控器调用的服务封装成http调用，但本次为流式调用，看看如何暴露这个http服务比较合适。
上下文交代：
1.moveit的流式实时控制servo已经配置完毕并启动测试成功，移动的调用可以参考src/dummy_server/server/STREAM_API_DOC.md
2.不要修改原来离线调用的api方式相关文件，例如api_server.py等，本次新增http服务server应该命名为stream_api_server.py

启动流式服务
cd ~/apps/dummy_ws
python src/dummy_server/server/stream_api_server.py（如果控制没反应，重复启用两次即可）
测试
python src/dummy_server/server/stream_api_client_demo.py（流式控制末端）
python src/dummy_server/server/stream_api_client_demo2.py（流式控制各个关节，查询关节状态，恢复关节）


--lerobot增加手机控制机械臂

注意：转到lerobot项目，记住激活conda的lerobot环境

参考examples/phone_to_so100/teleopmobile_test.py，同样使用mobile的应用控制机械臂。
但控制机械臂的操作不再直接调用CLI-Tool，而需要参考stream_api_client_demo.py中的http调用。
这次同样用到a123和a678，a1控制x即水平左右，a2控制y即水平前后，a3控制z即垂直上下，a6控制Pitch,a7控制Yaw，a8控制Roll
这次脚本写到新文件examples/phone_to_so100/stream_teleopmobile_test.py

修改stream_teleopmobile_test_v2.py，在保持原有功能不变的情况下，增加一些功能
参考以下调用demo：
python src/dummy_server/server/stream_api_client_demo.py（流式控制末端）（已经实现的api）
python src/dummy_server/server/stream_api_client_demo2.py（流式控制各个关节，查询关节状态，恢复关节）（将要实现的api）
这次增加功能有：
1.流式控制各个joint运动，当长按b3时触发（不按b3则保持控制末端坐标），a1a2控制joint1和2，a3控制joint3，a6控制joint6，a7a8控制joint4和5
2.恢复初始位姿，按b6时触发，调用api恢复初始位姿

包括调用：
1.流式控制末端坐标移动
2.流式控制各个joint运动
3.查询所有joint当前状态
4.恢复初始位姿


cd ~/apps/gemini/lerobot
python examples/phone_to_dummy/stream_teleopmobile_test_v2.py




--启动全流程

启动moveit中的机械臂
ros2 launch dummy_moveit_config servo_streaming.launch.py

启动moveit中的流式服务
cd ~/apps/dummy_ws
python src/dummy_server/server/stream_api_server.py（如果控制没反应，重复启用两次即可）

启动lerobot环境
source ~/apps/python/init_conda.profile
conda activate lerobot
cd ~/apps/gemini/lerobot

测试lerobot
python examples/phone_to_dummy/stream_teleopmobile_test_v2.py

启动遥操（robot.ip使用默认127.0.0.1）
lerobot-teleoperate     --robot.type=dummy_stream     --robot.id=my_awesome_arm     --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 15, fourcc: "MJPG"}, env: {type: intelrealsense, "serial_number_or_name": "242322075348", color_mode: "rgb", width: 640, height: 480, fps: 15, warmup_s: 10}}"     --teleop.type=mobile     --teleop.id=my_awesome_teleop    --fps=1
lerobot-teleoperate     --robot.type=dummy_stream     --robot.id=my_awesome_arm     --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}}"     --teleop.type=mobile     --teleop.id=my_awesome_teleop
lerobot-teleoperate     --robot.type=dummy_stream     --robot.id=my_awesome_arm     --robot.cameras="{env: {type: intelrealsense, "serial_number_or_name": "242322075348", width: 640, height: 480, fps: 15}}"     --teleop.type=mobile     --teleop.id=my_awesome_teleop
lerobot-teleoperate     --robot.type=dummy_stream     --robot.id=my_awesome_arm     --teleop.type=mobile     --teleop.id=my_awesome

安装依赖与查看设备
pip install -e ".[realsense]"
lerobot-find-cameras realsense

问题：
1.接了摄像头会有moveit调用卡顿和摄像头延迟的问题
解决方案一，都用realsense，realsense摄像头相对流畅一点点但也卡
解决方案二，movite服务与物理机械臂，跟lerobot环境与摄像头分开两台电脑




--使用virtualbox虚拟机 
--勉强可以维持半分钟
lerobot-teleoperate     --robot.type=dummy_stream     --robot.id=my_awesome_arm     --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 320, height: 240, fps: 30, fourcc: "MJPG"}, env: {type: opencv, index_or_path: 2, width: 320, height: 240, fps: 30, fourcc: "MJPG"}}"     --teleop.type=mobile     --teleop.id=my_awesome_teleop
--可以维持2分钟（没有启动moveit http服务都一样）
lerobot-teleoperate     --robot.type=dummy_stream     --robot.id=my_awesome_arm     --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 320, height: 240, fps: 10, fourcc: "MJPG"}, env: {type: opencv, index_or_path: 2, width: 320, height: 240, fps: 10, fourcc: "MJPG"}}"     --teleop.type=mobile     --teleop.id=my_awesome_teleop  --fps=10












