#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import threading

import numpy as np
from itertools import product

import csv

class PositionSpaceCounter(Node):
    def __init__(self):
        super().__init__('dummy_arm_space')
        self.pose_msg = None
        self.prev_pose = None  # cache the last pose
        self.lock = threading.Lock()
        self.results = []  # output result set
        self.space_count()

    def space_count(self):
        # Define range and step
        x_range = np.arange(-0.25, 0.25 , 0.03)
        y_range = np.arange(-0.50, 0.00 , 0.03)
        z_range = np.arange(-0.00, 0.50 , 0.03)
        # Generate all combinations in x, z, y order
        raw_points = product(x_range, y_range, z_range)  # xyz
        # Round all values to 2 decimal places
        points = [tuple(round(coord, 2) for coord in point) for point in raw_points]
        print('points matrix size：',len(points))
        for temp in points[:] :
            x = temp[0]
            y = temp[1]
            z = temp[2]
            valid = False
            # run target command
            try:
                result = subprocess.run([
                    'ros2', 'run', 'pymoveit2', 'pose_goal_server.py',
                    '--ros-args',
                    '-p', f'position:=[{x:.3f},{y:.3f},{z:.3f}]',
                    '-p', 'quat_xyzw:=[0.0,0.0,0.0,1.0]',
                    '-p', 'cartesian:=False',
                    '-p', 'synchronous:=False'
                ], capture_output=True, text=True, check=True)
                self.get_logger().info("ros2 command successfully executed.")
                self.get_logger().info(f"stdout: \n{result.stdout}")
                #self.get_logger().info(f"stderr: {result.stderr}")
                # check the trajectory result
                if 'MoveIt2State.REQUESTING' in result.stdout:
                    self.get_logger().info(f"yes,dummy can go there ,xyz=[{x:.3f},{y:.3f},{z:.3f}]")
                    valid = True
                elif 'MoveIt2State.IDLE' in result.stdout:
                    self.get_logger().info(f"no,dummy no way ,xyz=[{x:.3f},{y:.3f},{z:.3f}]")
                else:
                    self.get_logger().info(f"something wrong ,xyz=[{x:.3f},{y:.3f},{z:.3f}]")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"ros2 command failed: {e}")
            # add sample to result set
            self.results.append((x, y, z, valid))
        # export to csv file
        with open('result.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y', 'z', 'valid'])
            writer.writerows(self.results)

def main(args=None):
    rclpy.init(args=args)
    node = PositionSpaceCounter()
    #try:
    #    rclpy.spin(node)
    #except KeyboardInterrupt:
    #    pass
    #finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

