import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, WorkspaceParameters

from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

from tf_transformations import quaternion_from_euler

class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('moveit_server')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self):
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Waiting for MoveGroup action server...')
        
        # create MotionPlanRequest
        request = MotionPlanRequest()
        request.group_name = "dummy_arm" #dummy_arm  # change your move_group name
        request.allowed_planning_time = 3.0

        # set a target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.05  # meter, + is left and - is right
        target_pose.pose.position.y = -0.25  # meter, + is back and - is front 
        target_pose.pose.position.z = 0.30  # meter, + is up and - is down
        # angle euler value
        roll = 0.7  # rad, x, + face down and - face up
        pitch = 0.0  # rad, y, + twist left and - twist right
        yaw = 0.0  # rad, z, + turn left and - turn right
        q = quaternion_from_euler(roll, pitch, yaw)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # set goal constraint
        goal_constraint = Constraints()
        goal_constraint.name = "pose_goal"
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "link6_1_1"  # <<< Replace with your end effector link or last link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        # Position acurrency range constraint
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]  # small box around target
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0
        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "link6_1_1"  # <<< Replace with your end effector link or last link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.03
        ori_constraint.absolute_y_axis_tolerance = 0.03
        ori_constraint.absolute_z_axis_tolerance = 0.03
        ori_constraint.weight = 1.0
        # add constraint to request
        goal_constraint.position_constraints.append(pos_constraint)
        goal_constraint.orientation_constraints.append(ori_constraint)
        request.goal_constraints.append(goal_constraint)
        # Set workspace/bounders of the arm's moving area (optional but good practice)
        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = "base_link"
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0
        
        request.start_state.is_diff = True  # from current state

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.plan_only = False
        
        self.get_logger().info('Sending motion goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Motion execution result: {result.error_code}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MoveGroupClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

