import os
import sys

dirname = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dirname)

from classes.manipulation_parameters import Parameters


defaults = Parameters(
    ompl_planner="BITstar",
    step_size=0.04  # 0.001
)
