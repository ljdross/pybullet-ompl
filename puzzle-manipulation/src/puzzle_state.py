import pybullet as p
from configuration import Configuration


class PuzzleState:
    def __init__(self, world_id: int, config: Configuration):
        self.world_id = world_id
        self.puzzle_indices = []

        print("checking the following joints:")

        puzzle_base_index = len(config.robot_start_state)
        for i in range(puzzle_base_index, p.getNumJoints(self.world_id)):
            joint_info = p.getJointInfo(self.world_id, i)
            print(joint_info)
            if joint_info[2] == p.JOINT_REVOLUTE or joint_info[2] == p.JOINT_PRISMATIC:
                self.puzzle_indices.append(joint_info[0])

        print("puzzle_indices = " + str(self.puzzle_indices))

        for i, joint_pos in enumerate(config.puzzle_start_state):
            self.change_joint_pos(i, joint_pos)

    def change_joint_pos(self, joint_index: int, joint_pos: float):
        p.changeDynamics(self.world_id, self.puzzle_indices[joint_index], jointLowerLimit=joint_pos,
                         jointUpperLimit=joint_pos)
        p.resetJointState(self.world_id, self.puzzle_indices[joint_index], joint_pos)
