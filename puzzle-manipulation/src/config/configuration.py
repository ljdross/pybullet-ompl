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
    robot_x_limits: tuple[float, float]
    robot_y_limits: tuple[float, float]

    puzzle_urdf: str
    puzzle_start_state: tuple[float, ...]

    action_sequence: tuple[Action, ...]
    grab_action_height: float
    ompl_planner: str
    planning_time: float
    step_size: float
