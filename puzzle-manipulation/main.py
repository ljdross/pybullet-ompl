from src.manipulation import Manipulation
from src.configuration import Configuration
from src.action import Action


def main():
    config = Configuration(
        robot_path="/home/le/ba/pybullet-ompl/models/kuka_iiwa/model_in_2d_plane.urdf",
        puzzle_path="/home/le/ba/puzzle-generator/puzzles/simple_sliders/urdf/simple_sliders.urdf",
        merged_world_output_path="/home/le/ba/pybullet-ompl/models/kuka_iiwa/merged_world.urdf",
        robot_start_state=(-1, -1, 0, 0, 0, -1, 0, 1.5, 0),
        puzzle_start_state=(0, 0, 0, 0),
        action_sequence=(Action(3, 0.5), Action(2, 0.5), Action(1, 0.5), Action(0, 0.5)),
        ompl_planner="BITstar",
        step_size=0.002
    )
    manipulation = Manipulation(config)
    manipulation.plan()
    manipulation.execute()


if __name__ == '__main__':
    main()
