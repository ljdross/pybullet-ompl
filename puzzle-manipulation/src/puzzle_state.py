import pybullet as p


class PuzzleState:
    def __init__(self, body_id: int, puzzle_base_index: int, initial_state: tuple[float, ...]):
        self.puzzle_id = body_id
        self.puzzle_indices = []

        for i in range(puzzle_base_index, p.getNumJoints(self.puzzle_id)):
            joint_info = p.getJointInfo(self.puzzle_id, i)
            print(joint_info)
            if joint_info[2] == p.JOINT_REVOLUTE or joint_info[2] == p.JOINT_PRISMATIC:
                self.puzzle_indices.append(joint_info[0])

        print(self.puzzle_indices)

        self.state = initial_state
        for i, joint_pos in enumerate(initial_state):
            self.change_joint_pos(i, joint_pos)

    def change_joint_pos(self, joint_index: int, joint_pos: float):
        p.changeDynamics(self.puzzle_id, self.puzzle_indices[joint_index], jointLowerLimit=joint_pos,
                         jointUpperLimit=joint_pos)
        p.resetJointState(self.puzzle_id, self.puzzle_indices[joint_index], joint_pos)
