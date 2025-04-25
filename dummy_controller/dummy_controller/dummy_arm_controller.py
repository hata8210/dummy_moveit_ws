import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import numpy as np
import dummy_controller.dummy_cli_tool.ref_tool

class JointTrajectoryActionServer(Node):
    
    my_driver = None
    rad_volumn_diff = np.array([0,0,1.57079,0,0,0])
    rad_direct_diff = np.array([1,1,1,1,-1,1])
    pai = 3.1415926
    ready_rad = np.array([0,0,0,0,0,0])
    home_rad = np.array([0,-1.3089,1.5707,0,0,0])

    def __init__(self):
        super().__init__('dummy_arm_controller_real')
        self.get_logger().info('Ready to setup dummy arm')
        self.my_driver = dummy_controller.dummy_cli_tool.ref_tool.find_any()
        self.my_driver.robot.set_enable(1)
        self.my_driver.robot.set_rgb_mode(4)  #green light is ready
        self.move_rad(self.ready_rad)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'dummy_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )

    def rad_fix(self,arr_rad):
        return (arr_rad+self.rad_volumn_diff)*self.rad_direct_diff

    def rad2degree(self,arr_rad):
        arr_degree = arr_rad/self.pai*180
        return arr_degree

    def degree2rad(self,arr_degree):
        arr_rad = arr_degree/180*self.pai
        return arr_rad

    def move_rad(self,arr_rad):
        arr_rad = self.rad_fix(arr_rad)
        arr_degree = self.rad2degree(arr_rad)
        self.my_driver.robot.move_j(arr_degree[0],arr_degree[1],arr_degree[2],arr_degree[3],arr_degree[4],arr_degree[5])
        return True

    async def execute_callback(self, goal_handle):
        self.get_logger().info('okok,Received trajectory goal.')
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points
        start_time = time.time()
        for idx, point in enumerate(points):
            target_positions = point.positions
            time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            # wait for time
            now = time.time()
            wait_time = start_time + time_from_start - now
            if wait_time > 0:
                time.sleep(wait_time)
            # sent to hardware (joint_names, target_positions)
            self.get_logger().info(f'[{idx}] Sending positions: {target_positions}')
            np_target_positions = np.array(target_positions)
            self.move_rad(np_target_positions)
        self.get_logger().info('Trajectory execution complete.')
        # execute succeed
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

    def cleanup(self):
        self.move_rad(self.home_rad)
        #self.my_driver.robot.set_enable(0)
        self.my_driver.robot.set_rgb_mode(0)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

