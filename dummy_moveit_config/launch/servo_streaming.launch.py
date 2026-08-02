import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_params = (
        ParameterBuilder("dummy_moveit_config")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/servo_streaming.yaml",
        )
        .to_dict()
    )

    # A node to publish world -> base_link transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # The servo node from MoveIt 2
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # Dummy Servo Hardware Streamer
    # This node subscribes to /servo_node/command and publishes to /joint_states
    hardware_node = Node(
        package="dummy_controller",
        executable="dummy_servo_hardware",
        output="screen",
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("dummy_moveit_config"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit_empty.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    return LaunchDescription(
        [rviz_node, static_tf, servo_node, hardware_node, robot_state_publisher]
    )
