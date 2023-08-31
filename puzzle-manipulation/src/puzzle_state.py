import pybullet as p
from configuration import Configuration


class PuzzleState:
    def __init__(self, world_id: int, config: Configuration):
        self.world_id = world_id
        self.grip_points = self.get_grip_points(config)

        puzzle_base_index = len(config.robot_start_state)
        puzzle_joints = self.get_all_puzzle_joints(puzzle_base_index, p.getNumJoints(self.world_id))
        for joint in puzzle_joints:
            print(joint)

        self.puzzle_indices = self.get_puzzle_indices(puzzle_joints)
        print("puzzle_indices = " + str(self.puzzle_indices))

        self.joint_index_map = self.get_joint_index_map(puzzle_joints)
        print("grip_points_index_map = " + str(self.joint_index_map))

        for i, joint_pos in enumerate(config.puzzle_start_state):
            self.change_joint_pos(i, joint_pos)

    def get_joint_index_map(self, puzzle_joints):
        grip_points_index_map = {}
        for joint in puzzle_joints:
            name = str(joint[1]).removeprefix("b'").removesuffix("'")
            grip_points_index_map.update({name: joint[0]})
        return grip_points_index_map

    def get_puzzle_indices(self, puzzle_joints):
        puzzle_indices = []
        for joint in puzzle_joints:
            joint_type = joint[2]
            if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
                puzzle_indices.append(joint[0])
        return puzzle_indices

    def get_grip_points(self, config):
        grip_points = set()
        for action in config.action_sequence:
            grip_points.add(action.grip_point)
        return grip_points

    def get_all_puzzle_joints(self, min_index, max_index):
        joints = []
        for i in range(min_index, max_index):
            joints.append(p.getJointInfo(self.world_id, i))
        return joints

    def change_joint_pos(self, joint_index: int, joint_pos: float):
        p.changeDynamics(self.world_id, self.puzzle_indices[joint_index], jointLowerLimit=joint_pos,
                         jointUpperLimit=joint_pos)
        p.resetJointState(self.world_id, self.puzzle_indices[joint_index], joint_pos)

    def get_joint_pos(self, joint_index: int):
        return p.getJointState(self.world_id, self.puzzle_indices[joint_index])[0]

    def move_joint(self, joint_index: int, amount: float):
        current = self.get_joint_pos(joint_index)
        self.change_joint_pos(joint_index, current + amount)
