from dataclasses import dataclass


@dataclass
class Parameters:
    ompl_planner: str
    step_size: float
