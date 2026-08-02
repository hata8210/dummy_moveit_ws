#!/usr/bin/env python3
import requests
import time
import json

BASE_URL = "http://127.0.0.1:8000"

def print_response(title, response):
    print(f"\n--- {title} ---")
    print(f"Status Code: {response.status_code}")
    try:
        print(json.dumps(response.json(), indent=2))
    except json.JSONDecodeError:
        print(response.text)

def check_status():
    response = requests.get(f"{BASE_URL}/api/status")
    print_response("1. Checking Robot Status", response)
    return response.json()

def check_pose():
    response = requests.get(f"{BASE_URL}/api/pose")
    print_response("2. Checking Current End-Effector Pose", response)
    return response.json()

def move_to_pose(x, y, z):
    payload = {
        "x": x, "y": y, "z": z,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        "cartesian": False
    }
    response = requests.post(f"{BASE_URL}/api/move_to_pose", json=payload)
    print_response(f"3. Standard Move to (X:{x}, Y:{y}, Z:{z})", response)
    return response.json()

def move_optimal(x, y, z, start_joints=None):
    payload = {
        "x": x, "y": y, "z": z,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
    }
    if start_joints:
        payload["start_joints"] = start_joints
        
    response = requests.post(f"{BASE_URL}/api/move_optimal", json=payload)
    title_suffix = "with custom seed" if start_joints else "with current real joints"
    print_response(f"4. Optimal Move to (X:{x}, Y:{y}, Z:{z}) [{title_suffix}]", response)
    return response.json()

def wait_for_idle():
    print("\n[Waiting for robot to finish moving...]")
    while True:
        try:
            status = requests.get(f"{BASE_URL}/api/status").json()
            last_result = status.get("last_action_result", {})
            action_status = last_result.get("status", "none")
            
            # 只有当机器人状态为 IDLE，并且后台任务不在 running 状态时，才算真正结束
            if status.get("robot_state") == "IDLE" and action_status != "running":
                print("-> Robot is IDLE and ready.")
                print(f"-> [ACTION RESULT]: {action_status.upper()} - {last_result.get('message', '')}")
                break
                
            print(f"-> Robot is moving... (State: {status.get('robot_state')}, Action: {action_status})")
            time.sleep(1)
        except Exception as e:
            print(f"Error checking status: {e}")
            break

if __name__ == "__main__":
    print("=========================================")
    print("  Dummy Arm MoveIt 2 API - Client Demo  ")
    print("=========================================")
    
    # 0. 检查连接
    try:
        ##INFO:     127.0.0.1:57554 - "GET /api/pose HTTP/1.1" 200 OK
        #INFO:     127.0.0.1:57558 - "POST /api/move_optimal HTTP/1.1" 2 
        requests.get(f"{BASE_URL}/api/status")
    except requests.exceptions.ConnectionError:
        #INFO:     127.0.0.1:57554 - "GET /api/pose HTTP/1.1" 200 OK
        #INFO:     127.0.0.1:57558 - "POST /api/move_optimal HTTP/1.1" 2
        print(f"ERROR: Could not connect to {BASE_URL}.")
        print("Please start the ROS 2 API Server first using: ros2 run pymoveit2 api_server.py")
        exit(1)

    # 1. 查询机器人状态 (GET /api/status)
    print("\n[Step 1] 测试获取机器人状态 API...")
    check_status()
    

    # 2. 查询当前末端位姿 (GET /api/pose)
    print("\n[Step 2] 测试获取末端位姿 API...")
    check_pose()
    
    # =================================================================
    # ⚠️ 警告：以下代码会控制真实的机械臂运动！
    # 为了安全起见，默认被注释。如需观察机械臂运动，请取消对应代码块的注释。
    # =================================================================
    
    
    print("\n>>> 开始执行动作测试...")
    
    # 3. 标准位姿规划 (POST /api/move_to_pose)
    # 此 API 让 MoveIt 自由规划到达目标位姿的路径
    print("\n[Step 3] 测试标准位姿规划 API...")
    move_to_pose(0.0, -0.29, 0.20)
    wait_for_idle()
    check_pose() # 到达后再次检查位姿
    
    
    # 4. 最优关节规划 - 基于当前状态就近规划 (POST /api/move_optimal)
    # 不传 start_joints，系统会自动拉取物理机械臂当前的真实角度作为寻找逆解的种子
    print("\n[Step 4] 测试最优关节规划 API (基于当前真实状态)...")
    move_optimal(0.0, -0.29, 0.25)
    wait_for_idle()
    check_pose()
    
    # 5. 最优关节规划 - 基于自定义虚拟起点进行规划 (POST /api/move_optimal)
    # 传入自定义的 start_joints，强行让 MoveIt 认为机械臂是从这个虚拟姿态开始寻找解的
    #print("\n[Step 5] 测试最优关节规划 API (基于自定义虚拟起点)...")
    #virtual_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #move_optimal(0.15, -0.1, 0.30, start_joints=virtual_start)
    #wait_for_idle()
    #check_pose()
    
    
    print("\nAPI Demo 执行完毕。如需测试真实运动，请在脚本中取消对应代码块的注释。")
