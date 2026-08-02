#!/usr/bin/python3
from fastapi import FastAPI, BackgroundTasks
from pydantic import BaseModel
import uvicorn
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
import threading
import time
import math

app = FastAPI(title="Dummy Arm MoveIt 2 Stream API")

# Global variables
ros_node = None
servo_pub = None
jog_pub = None
executor = None
executor_thread = None

# States
current_mode = 'twist'  # 'twist' or 'joint'
current_joints = {}
initial_joints = None

# Current twist command state
current_command = {
    "linear_x": 0.0,
    "linear_y": 0.0,
    "linear_z": 0.0,
    "angular_x": 0.0,
    "angular_y": 0.0,
    "angular_z": 0.0,
    "last_update": 0.0
}

# Current joint command state
current_joint_command = {
    "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "last_update": 0.0
}

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

class StreamCommand(BaseModel):
    """Cartesian stream command"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

class JointStreamCommand(BaseModel):
    """Joint velocity stream command. Expected values between -1.0 and 1.0"""
    v1: float = 0.0
    v2: float = 0.0
    v3: float = 0.0
    v4: float = 0.0
    v5: float = 0.0
    v6: float = 0.0

def joint_state_callback(msg: JointState):
    global current_joints, initial_joints
    
    # Update current joints mapping
    for i, name in enumerate(msg.name):
        if name in JOINT_NAMES:
            current_joints[name] = msg.position[i]
            
    # Record initial joints if we haven't yet and we have all 6
    if initial_joints is None and len(current_joints) >= 6:
        initial_joints = {name: current_joints[name] for name in JOINT_NAMES}
        if ros_node:
            ros_node.get_logger().info(f"Recorded initial joint angles: {initial_joints}")

def publish_loop():
    """Background thread to publish twist/joint commands at 50Hz for MoveIt Servo."""
    global ros_node, servo_pub, jog_pub, current_command, current_joint_command, current_mode
    
    # Wait for ros_node to be initialized
    while ros_node is None and rclpy.ok():
        time.sleep(0.1)
        
    ros_node.get_logger().info("Starting 50Hz publish loop...")
    rate = ros_node.create_rate(50)
    
    while rclpy.ok():
        twist_active = (time.time() - current_command["last_update"]) <= 0.5
        joint_active = (time.time() - current_joint_command["last_update"]) <= 0.5
        
        # Determine what to publish based on mode and watchdogs
        if twist_active and current_mode == 'twist':
            msg = TwistStamped()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = current_command["linear_x"] * 0.05
            msg.twist.linear.y = current_command["linear_y"] * 0.05
            msg.twist.linear.z = current_command["linear_z"] * 0.05
            msg.twist.angular.x = current_command["angular_x"] * 0.1
            msg.twist.angular.y = current_command["angular_y"] * 0.1
            msg.twist.angular.z = current_command["angular_z"] * 0.1
            servo_pub.publish(msg)
            
        elif joint_active and current_mode == 'joint':
            msg = JointJog()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.joint_names = JOINT_NAMES
            # Scale joint input slightly if desired, or pass directly.
            # Using 0.5 scale max for safety, though user tested 0.1, 0.2
            # Let's pass them directly scaled by a fixed max velocity (e.g. 0.5 rad/s)
            msg.velocities = [v * 0.5 for v in current_joint_command["velocities"]]
            jog_pub.publish(msg)
            
        else:
            # Watchdog triggered or nothing active. Publish zeros to both to guarantee stop.
            twist_msg = TwistStamped()
            twist_msg.header.stamp = ros_node.get_clock().now().to_msg()
            twist_msg.header.frame_id = "base_link"
            servo_pub.publish(twist_msg)
            
            # Note: Do not spam zero JointJog aggressively to avoid conflict,
            # but publishing zero twist usually preempts and stops motion.
            
        try:
            rate.sleep()
        except Exception:
            break

@app.on_event("startup")
def startup_event():
    global ros_node, servo_pub, jog_pub, executor, executor_thread
    
    if not rclpy.ok():
        rclpy.init()
        
    ros_node = Node("stream_api_server")
    
    # Subscriptions
    ros_node.create_subscription(JointState, "/joint_states", joint_state_callback, 10)
    
    # Publishers
    servo_pub = ros_node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    jog_pub = ros_node.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
    
    # === Initialize Servo ===
    import subprocess
    ros_node.get_logger().info("Initializing servo via ROS 2 CLI commands...")
    
    try:
        # 1. Start Servo Service
        srv_cmd = ["ros2", "service", "call", "/servo_node/start_servo", "std_srvs/srv/Trigger"]
        ros_node.get_logger().info(f"Calling service: {' '.join(srv_cmd)}")
        res_srv = subprocess.run(srv_cmd, capture_output=True, text=True, check=True, timeout=10)
        
        if "success=True" not in res_srv.stdout and "success=true" not in res_srv.stdout.lower():
            ros_node.get_logger().error(f"Servo start failed. Stdout: {res_srv.stdout}\nStderr: {res_srv.stderr}")
            raise RuntimeError(f"Service /servo_node/start_servo failed to return success=True.\nOutput: {res_srv.stdout}")
        
        ros_node.get_logger().info("Servo started successfully (success=True verified).")
        
        # Wait 5 seconds
        ros_node.get_logger().info("Waiting 5 seconds for servo to fully initialize...")
        time.sleep(5)
        
        # 2. Publish initial delta joint commands
        ros_node.get_logger().info("Publishing initial JointJog commands natively...")
        
        wait_timeout = 5.0
        start_wait = time.time()
        while jog_pub.get_subscription_count() == 0 and rclpy.ok():
            if time.time() - start_wait > wait_timeout:
                ros_node.get_logger().warn("Timeout waiting for /servo_node/delta_joint_cmds subscriber. Publishing anyway.")
                break
            time.sleep(0.1)
            
        jog_msg = JointJog()
        jog_msg.header.stamp = ros_node.get_clock().now().to_msg()
        jog_msg.header.frame_id = "base_link"
        jog_msg.joint_names = JOINT_NAMES
        jog_msg.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        
        for _ in range(3):
            jog_pub.publish(jog_msg)
            time.sleep(0.1)
            
        ros_node.get_logger().info("Initial joint commands published successfully via rclpy.")
        
    except Exception as e:
        ros_node.get_logger().error(f"Servo initialization error: {e}")
        raise RuntimeError(f"Servo initialization error: {e}")
    # ========================

    # ROS executor thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ros_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Publisher loop thread
    pub_thread = threading.Thread(target=publish_loop, daemon=True)
    pub_thread.start()
    
    ros_node.get_logger().info("Stream API Server started on port 8001")

@app.on_event("shutdown")
def shutdown_event():
    global ros_node, executor_thread
    if ros_node:
        ros_node.get_logger().info("Shutting down Stream API Server...")
    if rclpy.ok():
        rclpy.shutdown()

@app.post("/api/stream")
def stream_control(cmd: StreamCommand):
    """Endpoint for Cartesian streaming control."""
    global current_command, current_mode
    current_mode = 'twist'
    
    current_command["linear_x"] = cmd.x
    current_command["linear_y"] = -cmd.y
    current_command["linear_z"] = cmd.z
    current_command["angular_x"] = cmd.roll
    current_command["angular_y"] = cmd.pitch
    current_command["angular_z"] = cmd.yaw
    current_command["last_update"] = time.time()
    
    return {"status": "ok", "mode": "twist"}

@app.post("/api/joint_stream")
def joint_stream_control(cmd: JointStreamCommand):
    """Endpoint for Joint velocity streaming control."""
    global current_joint_command, current_mode
    current_mode = 'joint'
    
    current_joint_command["velocities"] = [cmd.v1, cmd.v2, cmd.v3, cmd.v4, cmd.v5, cmd.v6]
    current_joint_command["last_update"] = time.time()
    
    return {"status": "ok", "mode": "joint"}

@app.get("/api/joints")
def get_joints():
    """Returns the current state of all 6 joints."""
    if not current_joints:
        return {"status": "error", "message": "Joint states not yet received."}
    return {
        "status": "ok",
        "joints": {name: current_joints.get(name, 0.0) for name in JOINT_NAMES}
    }

def restore_initial_task():
    """Background task to drive joints to their initial state using the joint_stream logic."""
    global current_mode, current_joint_command
    
    if not initial_joints:
        return
        
    ros_node.get_logger().info("Starting restore to initial joints...")
    current_mode = 'joint'
    
    # Simple P-controller loop to move joints back to initial
    rate = ros_node.create_rate(50) # 50 Hz control loop
    kp = 10.0 # Proportional gain (increased for very fast restore, 10x original)
    
    while rclpy.ok():
        max_error = 0.0
        velocities = [0.0] * 6
        
        for i, name in enumerate(JOINT_NAMES):
            if name in current_joints and name in initial_joints:
                err = initial_joints[name] - current_joints[name]
                if abs(err) > abs(max_error):
                    max_error = err
                
                # Calculate velocity cmd and clamp to [-1.0, 1.0]
                v = err * kp
                velocities[i] = max(-1.0, min(1.0, v))
                
        # If all joints are close to initial, stop
        if abs(max_error) < 0.02:  # ~1.1 degrees tolerance
            ros_node.get_logger().info("Restore complete. Reached initial joints.")
            break
            
        # Update command (like calling /api/joint_stream)
        current_joint_command["velocities"] = velocities
        current_joint_command["last_update"] = time.time()
        
        try:
            rate.sleep()
        except Exception:
            break

@app.post("/api/restore_initial")
def restore_initial(background_tasks: BackgroundTasks):
    """Triggers the robot to return to its initial joint angles."""
    if not initial_joints:
        return {"status": "error", "message": "Initial joints were not recorded yet."}
        
    background_tasks.add_task(restore_initial_task)
    return {"status": "accepted", "message": "Restoring to initial joints in background."}

@app.get("/api/status")
def get_status():
    """Returns the current command state and watchdog status."""
    twist_active = (time.time() - current_command["last_update"]) < 0.5
    joint_active = (time.time() - current_joint_command["last_update"]) < 0.5
    
    return {
        "status": "active" if (twist_active or joint_active) else "idle",
        "mode": current_mode,
        "watchdog_seconds_left": max(0, 0.5 - (time.time() - max(current_command["last_update"], current_joint_command["last_update"])))
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
