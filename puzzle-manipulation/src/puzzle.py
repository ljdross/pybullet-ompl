import pybullet as p
from configuration import Configuration


class Puzzle:
    def __init__(self, pb_id: int, config: Configuration):
        self.id = pb_id
        self.solution = []
        self.current_state = list(config.puzzle_start_state)

        puzzle_joints = self.get_all_puzzle_joints()

        self.puzzle_indices = self.get_puzzle_indices(puzzle_joints)
        print("puzzle_indices = " + str(self.puzzle_indices))

        self.joint_index_map = self.get_joint_index_map(puzzle_joints)
        print("grip_points_index_map = " + str(self.joint_index_map))

        self.set_state(self.current_state)

    def set_state(self, state):
        for i, joint_pos in enumerate(state):
            self.change_joint_pos(i, joint_pos)

    def get_joint_index_map(self, puzzle_joints):
        grip_points_index_map = {}
        for joint in puzzle_joints:
            name = str(joint[1]).removeprefix("b'").removesuffix("'")
            index = joint[0]
            grip_points_index_map.update({name: index})
        return grip_points_index_map

    def get_puzzle_indices(self, puzzle_joints):
        puzzle_indices = []
        for joint in puzzle_joints:
            joint_type = joint[2]
            if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
                puzzle_indices.append(joint[0])
        return puzzle_indices

    def get_all_puzzle_joints(self):
        joints = []
        for i in range(p.getNumJoints(self.id)):
            joint_info = p.getJointInfo(self.id, i)
            joints.append(joint_info)
        return joints

    def change_joint_pos(self, joint_index: int, joint_pos: float, debug=False):
        rounded_joint_pos = round(joint_pos, 5)
        self.current_state[joint_index] = rounded_joint_pos
        if debug:
            print("new puzzle state = " + str(self.current_state))

        p.changeDynamics(self.id, self.puzzle_indices[joint_index], jointLowerLimit=rounded_joint_pos,
                         jointUpperLimit=rounded_joint_pos)
        p.resetJointState(self.id, self.puzzle_indices[joint_index], rounded_joint_pos)

    def get_joint_pos(self, joint_index: int):
        return self.current_state[joint_index]

    def move_joint(self, joint_index: int, amount: float):
        current = self.get_joint_pos(joint_index)
        self.change_joint_pos(joint_index, current + amount)

    def add_current_state_to_solution(self, times: int):
        current = self.current_state.copy()
        self.solution.extend([current] * times)
