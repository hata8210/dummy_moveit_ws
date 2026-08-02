#!/usr/bin/env python3
"""
不依赖 ROS 2 环境的纯 Python 流式控制 API 调用 Demo
用于模拟遥控器、摇杆等设备向 Stream API Server 发送指令。
"""

import requests
import time

# 假设你的流式API服务器IP为 127.0.0.1，端口为 8001 (请根据实际情况修改IP)
STREAM_API_URL = "http://127.0.0.1:8001/api/stream"

def send_joystick_command(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """
    发送遥控器杆量 (值域: -1.0 到 1.0)
    x: 左右 (正为左, 负为右)
    y: 前后 (正为前, 负为后)
    z: 上下 (正为上, 负为下)
    roll: 前倾/后仰 (正为前倾, 映射至 angular.x)
    pitch: 旋左/旋右 (正为旋左, 映射至 angular.y)
    yaw: 摇左/摇右 (正为摇左, 映射至 angular.z)
    """
    payload = {
        "x": x,
        "y": y,
        "z": z,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw
    }
    try:
        # 设置较短的超时时间，保证流式控制的非阻塞特性
        response = requests.post(STREAM_API_URL, json=payload, timeout=0.2)
        if response.status_code == 200:
             # print(f"指令发送成功: {payload}")
             pass
        else:
             print(f"指令发送失败: 状态码 {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"网络请求错误 (请确保 stream_api_server.py 已经启动): {e}")

if __name__ == "__main__":
    print("=== 开始流式 API 调用 Demo ===")
    
    print("\n--- 场景 1: 单一方向移动 (仅水平向前) ---")
    for i in range(5):
        send_joystick_command(y=1.0)
        print(f"[{i+1}/5] 发送向前指令 y=1.0")
        time.sleep(0.1)

    print("\n--- 场景 2: 多方向同时移动 (向前 + 向左 = 左前斜向移动) ---")
    for i in range(5):
        send_joystick_command(x=1.0, y=1.0)
        print(f"[{i+1}/5] 发送左前斜向指令 x=1.0, y=1.0")
        time.sleep(0.1)

    print("\n--- 场景 3: 姿态控制 - Roll (前倾) ---")
    for i in range(5):
        send_joystick_command(roll=1.0)
        print(f"[{i+1}/5] 发送前倾指令 roll=1.0")
        time.sleep(0.1)

    print("\n--- 场景 4: 姿态控制 - Pitch (左旋) ---")
    for i in range(5):
        send_joystick_command(pitch=1.0)
        print(f"[{i+1}/5] 发送左旋指令 pitch=1.0")
        time.sleep(0.1)

    print("\n--- 场景 5: 姿态控制 - Yaw (右摇) ---")
    for i in range(50):
        send_joystick_command(yaw=-1.0)
        print(f"[{i+1}/5] 发送右摇指令 yaw=-1.0")
        time.sleep(0.1)

    print("\n--- 场景 6: 复杂的组合动作 (上升 + 向左斜移 + 后仰) ---")
    # Z轴拉起同时左移，并伴随一个后仰动作
    for i in range(8):
        send_joystick_command(z=1.0, x=0.5, roll=-0.5)
        print(f"[{i+1}/8] 发送复合指令 z=1.0, x=0.5, roll=-0.5")
        time.sleep(0.1)

    print("\n--- 场景 7: 停止移动 ---")
    send_joystick_command(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
    print("遥控器已归中发送全 0，机械臂立即停止。")
    print("\n=== Demo 运行结束 ===")
