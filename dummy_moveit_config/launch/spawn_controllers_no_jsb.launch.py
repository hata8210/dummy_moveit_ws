from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # 只启动真实关节控制器，不启动 joint_state_broadcaster
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dummy_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    )

    return ld


