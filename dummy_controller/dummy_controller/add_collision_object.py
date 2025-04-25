import sys
import time

import rclpy
from rclpy.node import Node
from moveit_commander import PlanningSceneInterface, roscpp_initialize
from geometry_msgs.msg import PoseStamped


class AddCollisionObject(Node):
    def __init__(self):
        super().__init__('add_collision_object_node')

        # init moveit_commander(ROS 1 style, still work at ros2)
        roscpp_initialize(sys.argv)

        # init scene
        self.scene = PlanningSceneInterface()

        # wait for the scene ready
        self.get_logger().info("Waiting for PlanningSceneInterface to be ready...")
        time.sleep(2)

        # add object
        self.add_box()

    def add_box(self):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'base_link'  # "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.2

        box_name = "box1"
        box_size = (0.4, 0.4, 0.4)  # box size

        self.scene.add_box(box_name, box_pose, size=box_size)
        self.get_logger().info(f"Added box '{box_name}' at position ({box_pose.pose.position.x}, "
                               f"{box_pose.pose.position.y}, {box_pose.pose.position.z})")

        # wait for appare in scene
        self.wait_for_box(box_name)

    def wait_for_box(self, box_name, timeout=5.0):
        start = time.time()
        while (time.time() - start < timeout) and not box_name in self.scene.get_known_object_names():
            time.sleep(0.1)
        if box_name in self.scene.get_known_object_names():
            self.get_logger().info(f"Confirmed box '{box_name}' added to the scene.")
        else:
            self.get_logger().warn(f"Box '{box_name}' not found in the scene after {timeout} seconds.")


def main(args=None):
    rclpy.init(args=args)
    node = AddCollisionObject()
    rclpy.spin_once(node, timeout_sec=2.0)  # just once here
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

