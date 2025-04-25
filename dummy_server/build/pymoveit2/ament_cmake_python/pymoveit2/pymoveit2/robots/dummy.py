from typing import List

MOVE_GROUP_ARM: str = "dummy_arm"
# MOVE_GROUP_GRIPPER: str = "gripper"

# OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
# CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "Joint1",
        prefix + "Joint2",
        prefix + "Joint3",
        prefix + "Joint4",
        prefix + "Joint5",
        prefix + "Joint6",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "link6_1_1"


# def gripper_joint_names(prefix: str = "") -> List[str]:
#     return [
#         prefix + "finger_joint1",
#         prefix + "finger_joint2",
#     ]



