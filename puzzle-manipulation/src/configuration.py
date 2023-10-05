from dataclasses import dataclass

from action import Action


@dataclass
class Configuration:
    robot_urdf: str
    puzzle_urdf: str
    robot_start_state: tuple[float, ...]
    puzzle_start_state: tuple[float, ...]
    action_sequence: tuple[Action, ...]
    ompl_planner: str
    step_size: float
