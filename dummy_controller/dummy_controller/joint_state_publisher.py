"""
独立的关节状态发布器模块

该模块封装了 ROS2 关节状态发布功能，提供清晰的接口供其他节点使用。
遵循单一职责原则，便于复用和测试。
"""

from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from typing import List, Optional
import numpy as np


class JointStatePublisher:
    """
    关节状态发布器类
    
    负责将关节位置、速度、力矩等信息发布到 /joint_states 话题。
    使用标准的 sensor_msgs/JointState 消息类型。
    
    示例用法:
        publisher = JointStatePublisher(node, joint_names=['Joint1', 'Joint2'])
        publisher.publish(positions=[0.1, 0.2], velocities=[0.0, 0.0])
    """
    
    def __init__(self, node, joint_names: List[str], topic_name: str = '/joint_states', 
                 publish_rate: float = 100.0):
        """
        初始化关节状态发布器
        
        Args:
            node: ROS2 Node 实例，用于创建发布器和获取时钟
            joint_names: 关节名称列表，例如 ['Joint1', 'Joint2', 'Joint3', ...]
            topic_name: 发布话题名称，默认为 '/joint_states'
            publish_rate: 发布频率（Hz），默认 100Hz，与 ros2_control 的 update_rate 一致
        """
        self._node = node
        self._joint_names = joint_names
        self._num_joints = len(joint_names)
        
        # 配置 QoS 策略
        # MoveIt 需要实时数据，使用 BEST_EFFORT 可靠性策略
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建发布者
        self._publisher = node.create_publisher(
            JointState,
            topic_name,
            qos_profile
        )
        
        self._publish_rate = publish_rate
        self._node.get_logger().info(
            f'JointStatePublisher initialized: {self._num_joints} joints, '
            f'topic: {topic_name}, rate: {publish_rate} Hz'
        )
    
    def publish(self, positions, 
                velocities=None,
                efforts=None):
        """
        发布关节状态消息
        
        自动支持列表和 numpy 数组输入，无需手动转换。
        
        Args:
            positions: 关节位置（弧度），可以是列表或 numpy 数组，长度必须与 joint_names 一致
            velocities: 关节速度（弧度/秒），可选，可以是列表或 numpy 数组，默认为 None
            efforts: 关节力矩（N·m），可选，可以是列表或 numpy 数组，默认为 None
        """
        # 自动转换 numpy 数组为列表
        if isinstance(positions, np.ndarray):
            positions = positions.tolist()
        elif not isinstance(positions, (list, tuple)):
            self._node.get_logger().error(
                f'Positions must be a list, tuple, or numpy array, got {type(positions)}'
            )
            return
        
        if len(positions) != self._num_joints:
            self._node.get_logger().error(
                f'Position array length ({len(positions)}) does not match '
                f'number of joints ({self._num_joints})'
            )
            return
        
        # 创建 JointState 消息
        msg = JointState()
        # 设置时间戳
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        # 设置关节名称
        msg.name = list(self._joint_names)
        # 设置关节位置（必须）
        msg.position = list(positions)
        
        # 设置关节速度（可选）
        if velocities is not None:
            # 自动转换 numpy 数组为列表
            if isinstance(velocities, np.ndarray):
                velocities = velocities.tolist()
            elif not isinstance(velocities, (list, tuple)):
                self._node.get_logger().warn(
                    f'Velocities must be a list, tuple, or numpy array, got {type(velocities)}, skipping'
                )
                velocities = None
            
            if velocities is not None:
                if len(velocities) != self._num_joints:
                    self._node.get_logger().warn(
                        f'Velocity array length ({len(velocities)}) does not match '
                        f'number of joints ({self._num_joints}), skipping velocities'
                    )
                else:
                    msg.velocity = list(velocities)
        
        # 设置关节力矩（可选）
        if efforts is not None:
            # 自动转换 numpy 数组为列表
            if isinstance(efforts, np.ndarray):
                efforts = efforts.tolist()
            elif not isinstance(efforts, (list, tuple)):
                self._node.get_logger().warn(
                    f'Efforts must be a list, tuple, or numpy array, got {type(efforts)}, skipping'
                )
                efforts = None
            
            if efforts is not None:
                if len(efforts) != self._num_joints:
                    self._node.get_logger().warn(
                        f'Effort array length ({len(efforts)}) does not match '
                        f'number of joints ({self._num_joints}), skipping efforts'
                    )
                else:
                    msg.effort = list(efforts)
        
        # 发布消息
        self._publisher.publish(msg)
    
    @property
    def joint_names(self) -> List[str]:
        """获取关节名称列表"""
        return self._joint_names
    
    @property
    def num_joints(self) -> int:
        """获取关节数量"""
        return self._num_joints

