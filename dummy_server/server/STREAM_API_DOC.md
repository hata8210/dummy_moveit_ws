--通过moveit实现流式控制

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

--水平向前（发送半秒）
ros2 topic pub --rate 50 -t 25 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {stamp: 'now', frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: -0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
