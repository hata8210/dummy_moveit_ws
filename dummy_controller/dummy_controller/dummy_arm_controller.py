import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import numpy as np
import dummy_controller.dummy_cli_tool.ref_tool
from dummy_controller.joint_state_publisher import JointStatePublisher

class JointTrajectoryActionServer(Node):
    
    my_driver = None
    pai = 3.1415926
    rad_volumn_diff = np.array([0,0,1.57079,0,0,0])
    rad_direct_diff = np.array([1,1,1,1,-1,-1])
    # homing
    ready_rad = np.array([0,0,0,0,0,0])
    # resting
    home_rad = np.array([0,-1.3089,1.57079,0,0,0])
    
    # motor to joint offset
    motor_to_joint_offset = np.array([0, -75, 180, 0, 0, 0])

    # 关节名称列表，与 MoveIt 配置保持一致
    joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

    def __init__(self):
        super().__init__('dummy_arm_controller_real')
        self.get_logger().info('Ready to setup dummy arm')
        self.my_driver = dummy_controller.dummy_cli_tool.ref_tool.find_any()
        self.my_driver.robot.set_enable(1)
        self.my_driver.robot.set_rgb_mode(4)  #green light is ready
        self.move_rad(self.ready_rad)   # homing
        
        # 创建轨迹执行 action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'dummy_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # 创建关节状态发布器（独立的模块）
        self.joint_state_publisher = JointStatePublisher(
            node=self,
            joint_names=self.joint_names,
            topic_name='/joint_states',
            publish_rate=100.0  # 100Hz，与 ros2_control 的 update_rate 一致
        )
        
        # 创建定时器定期发布关节状态
        # 100Hz = 0.01秒间隔
        self.joint_state_timer = self.create_timer(0.01, self.publish_joint_states_callback)
        
        self.get_logger().info('Joint state publisher initialized and started')

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
        self.get_logger().info(f"move_rad:{arr_degree}")
        return True
    
    def read_joint_positions_from_hardware(self):
        """
        从硬件驱动读取当前关节位置
        
        硬件 API: robot.joint_1.angle 到 robot.joint_6.angle
        数据格式: 电机角度，直接读取
        
        返回:
            numpy.ndarray: 6个关节的位置（弧度），已应用坐标转换
        """
        try:
            robot = self.my_driver.robot
            
            # 从硬件读取关节角度（度）
            # 硬件 API: robot.joint_1.angle 到 robot.joint_6.angle
            positions_degree = np.array([
                robot.joint_1.angle,  # Joint1
                robot.joint_2.angle,  # Joint2
                robot.joint_3.angle,  # Joint3
                robot.joint_4.angle,  # Joint4
                robot.joint_5.angle,  # Joint5
                robot.joint_6.angle,  # Joint6
            ])
            
            # 将度转换为弧度
            positions_degree += self.motor_to_joint_offset
            positions_rad = self.degree2rad(positions_degree)
            
            positions_rad = (positions_rad / self.rad_direct_diff) - self.rad_volumn_diff
            return positions_rad
        except Exception as e:
            self.get_logger().error(f'Error in read_joint_positions_from_hardware: {e}', exc_info=True)
            return np.zeros(6)


    def publish_joint_states_callback(self):
        """
        定时器回调函数：定期发布关节状态
        
        这个函数会被定时器定期调用（100Hz），读取硬件关节位置并发布到 /joint_states 话题
        """
        try:
            # 从硬件读取当前关节位置
            positions = self.read_joint_positions_from_hardware()
            
            # 发布关节状态
            # 注意：目前只发布位置，速度和力矩可以后续添加
            # publish() 方法会自动处理 numpy 数组，无需手动转换
            self.joint_state_publisher.publish(
                positions=positions,
                velocities=None,  # 如果硬件支持读取速度，可以在这里添加
                efforts=None     # 如果硬件支持读取力矩，可以在这里添加
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in publish_joint_states_callback: {e}', exc_info=True)

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

