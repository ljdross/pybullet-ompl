from dataclasses import dataclass


@dataclass
class Action:
    joint_index: int
    joint_pos: float
