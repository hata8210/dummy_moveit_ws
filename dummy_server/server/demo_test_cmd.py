
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import dummy as robot

from rclpy.clock import Clock

#import subprocess

# Method 1: simple command execution with output
#result = subprocess.run(['ls', '-l'], capture_output=True, text=True)
#print(result.stdout)

# Method 2: with error handling
#try:
#    subprocess.run(['mkdir', 'test_dir'], check=True)
#    print("Directory created successfully")
#except subprocess.CalledProcessError as e:
#    print(f"Command failed: {e}")

def main():
    rclpy.init()
    try:
        #subprocess.run(['ros2','run','pymoveit2','pose_goal_server.py','--ros-args'
        #    ,'-p','position:=[0.0,-0.25,0.38]'
        #    ,'-p','quat_xyzw:=[0.0,0.0,0.0,1.0]'
        #    ,'-p','cartesian:=False','-p','synchronous:=False'], check=True)
        print("ros2 command successfully")
    except subprocess.CalledProcessError as e:
        print(f"ros2 command failed: {e}")
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()


