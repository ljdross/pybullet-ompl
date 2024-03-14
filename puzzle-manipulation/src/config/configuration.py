from dataclasses import dataclass


@dataclass
class Action:
    joint_index: int
    joint_pos: float
    grip_point: str


@dataclass
class Configuration:
    robot_urdf: str
    robot_scale: float
    robot_start_state: tuple[float, ...]
    robot_final_state: tuple[float, ...]

    puzzle_urdf: str
    puzzle_start_state: tuple[float, ...]

    action_sequence: tuple[Action, ...]
    ompl_planner: str
    step_size: float
