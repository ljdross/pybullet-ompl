from dataclasses import dataclass

from action import *


@dataclass
class Configuration:
    robot_path: str
    puzzle_path: str
    merged_world_output_path: str
    robot_start_state: tuple[float, ...]
    puzzle_start_state: tuple[float, ...]
    action_sequence: tuple[Action, ...]
    ompl_planner: str
    step_size: float
