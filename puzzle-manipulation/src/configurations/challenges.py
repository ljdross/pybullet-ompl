import os
import sys

dirname = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dirname)

from classes.configuration import Action, Configuration


grid_world = Configuration(
    robot_urdf="/home/le/ba/pybullet-ompl/models/kuka_iiwa/model_in_2d_plane.urdf",
    puzzle_urdf="/home/le/ba/puzzle-generator/puzzles/grid_world/urdf/grid_world.urdf",
    robot_start_state=(-4, -4, 0, 0, 0, -1, 0, 1.5, 0),  # TODO: make optional and sample a valid start state?
    puzzle_start_state=(0, 0, 0, 0),
    action_sequence=(
        Action(3, 1, "link_fixed_13_handle_knob"),
        Action(2, -1.5708, "link_fixed_10_handle_knob"),
        Action(1, 1, "link_fixed_6_handle_knob"),
        Action(0, 1.5708, "link_fixed_3_handle_knob")
    )
)


simple_sliders = Configuration(
    robot_urdf="/home/le/ba/pybullet-ompl/models/kuka_iiwa/model_in_2d_plane.urdf",
    puzzle_urdf="/home/le/ba/puzzle-generator/puzzles/simple_sliders/urdf/simple_sliders.urdf",
    robot_start_state=(-4, -4, 0, 0, 0, -1, 0, 1.5, 0),
    puzzle_start_state=(0, 0),
    action_sequence=(
        Action(1, 1, "link_fixed_5_handle_knob"),
        Action(0, 1, "link_fixed_2_handle_knob")
    )
)
