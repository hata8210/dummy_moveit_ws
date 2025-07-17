#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import threading
import csv
import numpy as np


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('track_aruco')
        # init space
        # set range
        x_min, x_max = -0.25, 0.25
        y_min, y_max = -0.5, -0.0
        z_min, z_max = 0.0, 0.5
        # collect space
        xs, ys, zs, colors = [], [], [], []
        with open('/home/hata_ros/apps/dummy_ws/src/dummy_controller/dummy_controller/'+'result.csv', newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                valid = row['valid'].lower() == 'true'
                color = 'green' if valid else 'red'
                if not (valid and x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
                    continue
                xs.append(x)
                ys.append(y)
                zs.append(z)
                colors.append(color)
        # save space
        self.points = np.column_stack((xs, ys, zs))
        # start subscribe
        self.pose_msg = None
        self.prev_pose = None  # cache the last pose
        self.lock = threading.Lock()
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_world_pose',
            self.pose_callback,
            1
        )
        # run tracker in every 5 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

    def find_nearest_point(self, x, y, z):
        """finde the nearest point by xyz position"""
        if self.points.size == 0:
            return None
        query_point = np.array([x, y, z])
        distances = np.linalg.norm(self.points - query_point, axis=1)
        nearest_index = np.argmin(distances)
        return tuple(self.points[nearest_index])

    def pose_callback(self, msg):
        with self.lock:
            self.pose_msg = msg
        self.get_logger().info(f'Received pose: position=({msg.pose.position.x:.3f}, '
                               f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})')

    def poses_equal(self, pose1, pose2, tol=1e-4):
        """compare tow pose is or not the same """
        if pose1 is None or pose2 is None:
            return False
        p1 = pose1.pose.position
        p2 = pose2.pose.position
        q1 = pose1.pose.orientation
        q2 = pose2.pose.orientation
        return (
            abs(p1.x - p2.x) < tol and
            abs(p1.y - p2.y) < tol and
            abs(p1.z - p2.z) < tol and
            abs(q1.x - q2.x) < tol and
            abs(q1.y - q2.y) < tol and
            abs(q1.z - q2.z) < tol and
            abs(q1.w - q2.w) < tol
        )

    def timer_callback(self):
        with self.lock:
            if self.pose_msg is None:
                self.get_logger().warn("No pose received yet.")
                return

            # check if Pose has not changed ,then skip
            if self.poses_equal(self.pose_msg, self.prev_pose):
                self.get_logger().info("Pose has not changed, skipping command.")
                return
            self.prev_pose = self.pose_msg

            x = self.pose_msg.pose.position.x
            y = self.pose_msg.pose.position.y+0.09+0.25 # add some distance about the camera in hand
            z = self.pose_msg.pose.position.z

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
            self.get_logger().info(f"stdout: \n{result.stdout}")
            if 'MoveIt2State.REQUESTING' not in result.stdout:
                nearest = self.find_nearest_point(x, y, z)
                x = nearest[0]
                y = nearest[1]
                z = nearest[2]
                self.get_logger().warn("replace by nearest point:"+str(x)+","+str(y)+","+str(z))
                result = subprocess.run([
                    'ros2', 'run', 'pymoveit2', 'pose_goal_server.py',
                    '--ros-args',
                    '-p', f'position:=[{x:.3f},{y:.3f},{z:.3f}]',
                    '-p', 'quat_xyzw:=[0.0,0.0,0.0,1.0]',
                    '-p', 'cartesian:=False',
                    '-p', 'synchronous:=False'
                    ], capture_output=True, text=True, check=True)
                self.get_logger().info(f"stdout: \n{result.stdout}")
            self.get_logger().info("ros2 command successfully executed.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"ros2 command failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

