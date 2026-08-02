#!/usr/bin/python3
from fastapi import FastAPI, BackgroundTasks
from pydantic import BaseModel
from typing import List, Optional
import uvicorn
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import dummy as robot
import threading

app = FastAPI(title="Dummy Arm MoveIt 2 API")

# Global variables for ROS 2 node and MoveIt 2 interface
ros_node = None
moveit_interface = None
executor = None
executor_thread = None
last_action_result = {"status": "none", "message": "No action executed yet."}

class PoseGoal(BaseModel):
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    cartesian: bool = False

class OptimalPoseGoal(BaseModel):
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    start_joints: Optional[List[float]] = None 

@app.on_event("startup")
def startup_event():
    global ros_node, moveit_interface, executor, executor_thread
    rclpy.init()
    ros_node = Node("dummy_api_server")
    callback_group = ReentrantCallbackGroup()
    
    moveit_interface = MoveIt2(
        node=ros_node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(ros_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    ros_node.get_logger().info("API Server ROS 2 Node started.")

@app.on_event("shutdown")
def shutdown_event():
    global ros_node, executor_thread
    if ros_node:
        ros_node.get_logger().info("Shutting down API Server ROS 2 Node...")
        rclpy.shutdown()
    if executor_thread:
        executor_thread.join(timeout=1.0)

@app.post("/api/move_to_pose")
def move_to_pose(goal: PoseGoal, background_tasks: BackgroundTasks):
    global moveit_interface, ros_node
    
    if not moveit_interface:
        return {"status": "error", "message": "MoveIt 2 interface not initialized."}
        
    state = moveit_interface.query_state()
    if state != MoveIt2State.IDLE:
        return {"status": "error", "message": f"Robot is currently {state.name}, cannot accept new goal."}

    def execute_motion():
        global last_action_result
        last_action_result = {"status": "running", "message": "Planning and executing motion..."}
        ros_node.get_logger().info(f"Executing API Goal: x={goal.x}, y={goal.y}, z={goal.z}")
        try:
            moveit_interface.move_to_pose(
                position=[goal.x, goal.y, goal.z],
                quat_xyzw=[goal.qx, goal.qy, goal.qz, goal.qw],
                cartesian=goal.cartesian
            )
            success = moveit_interface.wait_until_executed()
            if success:
                last_action_result = {"status": "success", "message": "Motion planned and executed successfully."}
                ros_node.get_logger().info("API Goal Execution Complete: Success.")
            else:
                last_action_result = {"status": "failed", "message": "Planning or execution failed."}
                ros_node.get_logger().warn("API Goal Execution Complete: Failed.")
        except Exception as e:
            last_action_result = {"status": "error", "message": f"Exception occurred: {str(e)}"}
            ros_node.get_logger().error(f"API Goal Error: {e}")

    background_tasks.add_task(execute_motion)
    return {"status": "accepted", "message": "Pose goal received and planning/execution started in background."}

@app.post("/api/move_optimal")
def move_optimal(goal: OptimalPoseGoal, background_tasks: BackgroundTasks):
    global moveit_interface, ros_node
    
    if not moveit_interface:
        return {"status": "error", "message": "MoveIt 2 interface not initialized."}
        
    state = moveit_interface.query_state()
    if state != MoveIt2State.IDLE:
        return {"status": "error", "message": f"Robot is currently {state.name}."}

    def execute_optimized_motion():
        global last_action_result
        last_action_result = {"status": "running", "message": "Preparing optimal motion planning..."}
        try:
            # 1. 确定用来做参考的起始关节状态 (Seed)
            seed_joints = None
            if goal.start_joints and len(goal.start_joints) == 6:
                seed_joints = goal.start_joints
                ros_node.get_logger().info(f"Using custom start joints as seed: {seed_joints}")
            else:
                # 如果没传，就获取机械臂真实的当前关节状态作为 Seed
                if moveit_interface._MoveIt2__joint_state is None:
                    last_action_result = {"status": "failed", "message": "Real joint state not available yet to be used as seed."}
                    ros_node.get_logger().error("Real joint state not available yet.")
                    return
                
                js_msg = moveit_interface._MoveIt2__joint_state
                name_to_pos = dict(zip(js_msg.name, js_msg.position))
                seed_joints = [name_to_pos[jname] for jname in robot.joint_names()]
                
                ros_node.get_logger().info("Using current real joints as seed.")

            # 2. 调用 IK 求逆解，传入 seed_joints 寻找最近的姿态
            last_action_result = {"status": "running", "message": "Computing Inverse Kinematics (IK)..."}
            ros_node.get_logger().info("Computing Inverse Kinematics (IK)...")
            target_joint_state = moveit_interface.compute_ik(
                position=[goal.x, goal.y, goal.z],
                quat_xyzw=[goal.qx, goal.qy, goal.qz, goal.qw],
                start_joint_state=seed_joints
            )
            
            if not target_joint_state:
                last_action_result = {"status": "failed", "message": "Planning failed: IK computation could not find a valid pose within reach."}
                ros_node.get_logger().error("IK computation failed. No valid pose found within reach.")
                return

            target_positions = list(target_joint_state.position)
            ros_node.get_logger().info(f"IK Found nearest joint configuration: {target_positions}")

            # 3. 如果你想让规划也是“从你传入的虚拟点开始”而不是真实点开始，
            # 需要修改 MoveIt 规划请求的起始点
            if goal.start_joints and len(goal.start_joints) == 6:
                 moveit_interface._MoveIt2__move_action_goal.request.start_state.joint_state.position = goal.start_joints
                 moveit_interface._MoveIt2__move_action_goal.request.start_state.joint_state.name = robot.joint_names()

            # 4. 执行关节空间规划 (Joint Space Planning)
            last_action_result = {"status": "running", "message": "Executing joint trajectory..."}
            ros_node.get_logger().info("Executing Joint Space trajectory...")
            moveit_interface.move_to_configuration(joint_positions=target_positions)
            success = moveit_interface.wait_until_executed()
            if success:
                last_action_result = {"status": "success", "message": "Optimal motion planned and executed successfully."}
                ros_node.get_logger().info("Optimal Goal Execution Complete: Success.")
            else:
                last_action_result = {"status": "failed", "message": "Execution of optimal trajectory failed."}
                ros_node.get_logger().warn("Optimal Goal Execution Complete: Failed.")
        except Exception as e:
            last_action_result = {"status": "error", "message": f"Exception occurred: {str(e)}"}
            ros_node.get_logger().error(f"Optimal Goal Error: {e}")

    background_tasks.add_task(execute_optimized_motion)
    return {"status": "accepted", "message": "Optimal planning started."}

@app.get("/api/status")
def get_status():
    global moveit_interface, last_action_result
    if not moveit_interface:
        return {"status": "error", "message": "Not initialized"}
    state = moveit_interface.query_state()
    current_joints = []
    if moveit_interface._MoveIt2__joint_state:
        js_msg = moveit_interface._MoveIt2__joint_state
        name_to_pos = dict(zip(js_msg.name, js_msg.position))
        current_joints = [name_to_pos.get(jname, 0.0) for jname in robot.joint_names()]
    return {
        "status": "ok", 
        "robot_state": state.name, 
        "current_joints": current_joints,
        "last_action_result": last_action_result
    }

@app.get("/api/pose")
def get_pose():
    global moveit_interface, ros_node
    if not moveit_interface:
        return {"status": "error", "message": "Not initialized"}
    
    # Check if joint state is available to compute FK
    if moveit_interface._MoveIt2__joint_state is None:
        return {"status": "error", "message": "Joint state not available yet to compute pose."}
        
    try:
        # compute_fk returns a PoseStamped for the end effector
        pose_stamped = moveit_interface.compute_fk()
        if pose_stamped is None:
            return {"status": "error", "message": "Failed to compute forward kinematics."}
            
        position = pose_stamped.pose.position
        orientation = pose_stamped.pose.orientation
        
        return {
            "status": "ok",
            "frame_id": pose_stamped.header.frame_id,
            "position": {
                "x": position.x,
                "y": position.y,
                "z": position.z
            },
            "orientation": {
                "x": orientation.x,
                "y": orientation.y,
                "z": orientation.z,
                "w": orientation.w
            }
        }
    except Exception as e:
        if ros_node:
            ros_node.get_logger().error(f"Error computing pose: {e}")
        return {"status": "error", "message": str(e)}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
