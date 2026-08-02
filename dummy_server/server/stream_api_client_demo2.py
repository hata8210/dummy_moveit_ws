#!/usr/bin/env python3
"""
流式控制 API 扩展功能测试 Demo 2
测试内容：
1. GET /api/joints - 查询实时关节状态
2. POST /api/joint_stream - 关节空间速度控制
3. POST /api/restore_initial - 自动恢复初始位姿
"""

import requests
import time

# 假设流式API服务器地址
BASE_URL = "http://127.0.0.1:8001/api"

def get_current_joints():
    """调用 GET /api/joints"""
    try:
        response = requests.get(f"{BASE_URL}/joints", timeout=1.0)
        if response.status_code == 200:
            data = response.json()
            if data["status"] == "ok":
                # 格式化输出，保留3位小数
                j_str = ", ".join([f"{name}: {pos:.3f}" for name, pos in data['joints'].items()])
                print(f"  [Joints] {j_str}")
                return data['joints']
        else:
            print(f"  查询关节失败: 状态码 {response.status_code}")
    except Exception as e:
        print(f"  请求异常: {e}")
    return None

def send_joint_stream(v1=0.0, v2=0.0, v3=0.0, v4=0.0, v5=0.0, v6=0.0):
    """调用 POST /api/joint_stream"""
    payload = {"v1": v1, "v2": v2, "v3": v3, "v4": v4, "v5": v5, "v6": v6}
    try:
        # 快速发送，超时设短
        requests.post(f"{BASE_URL}/joint_stream", json=payload, timeout=0.2)
    except Exception:
        pass

def trigger_restore():
    """调用 POST /api/restore_initial"""
    try:
        response = requests.post(f"{BASE_URL}/restore_initial", timeout=1.0)
        print(f"  触发复位响应: {response.json()}")
    except Exception as e:
        print(f"  触发复位失败: {e}")

if __name__ == "__main__":
    print("=== 开始流式 API 扩展功能测试 (Demo 2) ===")

    # 1. 查询初始状态
    print("\n[测试 1] 查询当前各关节角度 (rad)...")
    get_current_joints()
    time.sleep(1)

    # 2. 测试关节流式控制 (Joint Stream)
    print("\n[测试 2] 关节空间速度控制：让 Joint1(底座) 和 Joint2(大臂) 异向转动 3.0 秒 (加长轨迹)...")
    # 模拟遥控器 10Hz 频率发送
    for i in range(30):
        # 比例速度指令 [-1.0, 1.0]
        send_joint_stream(v1=0.8, v2=-0.5)
        if i % 5 == 0:
            print(f"  -> 正在发送关节速度指令... ({i+1}/30)")
        time.sleep(0.1)
    
    # 停止指令
    send_joint_stream(v1=0.0, v2=0.0)
    print("  -> 关节指令已清零停止。")
    time.sleep(1)
    
    # 3. 查看运动后的关节位置
    print("\n[测试 3] 运动停止后的实时关节角度：")
    get_current_joints()
    time.sleep(1)

    # 4. 测试一键复位 (Restore Initial)
    print("\n[测试 4] 触发一键复位 (自动寻找服务启动时的记录位置，速度提升5倍)...")
    trigger_restore()
    
    # 持续观察 8 秒复位过程，看角度是否在跳变回初始值
    print("  观察复位中的角度变化 (持续观察最多 8 秒)...")
    for i in range(16):
        get_current_joints()
        time.sleep(0.5)

    print("\n=== Demo 2 测试运行结束 ===")
