import math
import os
import sys
import time

import pybullet as p
import pybullet_data

dirname = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dirname)

import pb_ompl
from file_utils.urdf_merger import merge_urdf_files
from puzzle_state import PuzzleState
from configuration import Configuration
from action import Action


class Manipulation:
    def __init__(self, config: Configuration):
        self.config = config
        self.solution = []

        self.merge_robot_and_puzzle()
        self.set_up_pybullet_environment()
        world_id = self.load_world()
        self.puzzle_state = PuzzleState(world_id, self.config)
        self.pb_ompl_world, self.pb_ompl_interface = self.set_up_pb_ompl(world_id)
        self.print_world_info()

    def merge_robot_and_puzzle(self):
        merge_urdf_files(self.config.robot_path, self.config.puzzle_path, self.config.merged_world_output_path)

    def set_up_pybullet_environment(self):
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1. / 240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

    def load_world(self):
        orn = p.getQuaternionFromEuler((0, 0, 0))
        world_id = p.loadURDF(self.config.merged_world_output_path, (0, 0, 0), orn, useFixedBase=1,
                              flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)
        return world_id

    def set_up_pb_ompl(self, world_id):
        pb_ompl_world = pb_ompl.PbOMPLRobot(world_id)

        pb_ompl_interface = pb_ompl.PbOMPL(pb_ompl_world)
        pb_ompl_interface.set_planner(self.config.ompl_planner)

        start = list(self.config.robot_start_state + self.config.puzzle_start_state)
        pb_ompl_world.set_state(start)

        return pb_ompl_world, pb_ompl_interface

    def print_world_info(self):
        print("=" * 100)
        for i in range(p.getNumJoints(self.pb_ompl_world.id)):
            print(p.getJointInfo(self.pb_ompl_world.id, i))
        print("number of joints = " + str(p.getNumJoints(self.pb_ompl_world.id)))
        print("number of dimensions = " + str(self.pb_ompl_world.num_dim))
        print("=" * 100)

    def plan(self):
        for action in self.config.action_sequence:
            self.plan_action(action)

    def plan_action(self, action: Action):
        joint_index = self.puzzle_state.joint_index_map[action.grip_point]
        target_pos = self.get_target_pos(joint_index)
        target_pos_high = self.add_tuples(target_pos, (0, 0, 0.3))

        self.plan_and_move_to(target_pos_high)
        self.plan_and_move_to(target_pos, True)

        self.actuate(action)

        target_pos = self.get_target_pos(joint_index)
        target_pos_high = self.add_tuples(target_pos, (0, 0, 0.3))
        self.plan_and_move_to(target_pos_high, True)

    def get_target_pos(self, joint_index):
        link_state = p.getLinkState(self.pb_ompl_world.id, joint_index)
        pos = self.add_tuples(link_state[0], (0, 0, 0.15))
        return pos

    def actuate(self, action: Action):
        joint_index = self.puzzle_state.joint_index_map[action.grip_point]
        diff = self.calculate_diff(action)
        while abs(diff) > self.config.step_size:
            if diff < 0:
                self.puzzle_state.move_joint(action.joint_index, -self.config.step_size)
            else:
                self.puzzle_state.move_joint(action.joint_index, self.config.step_size)
            target_pos = self.get_target_pos(joint_index)
            self.plan_and_move_to(target_pos, True)
            diff = self.calculate_diff(action)


    def calculate_diff(self, action):
        target = action.joint_pos
        actual = self.puzzle_state.get_joint_pos(action.joint_index)
        return target - actual

    def plan_and_move_to(self, pos, double_speed=False):
        pos_state, is_valid = self.get_valid_state(pos)
        res, path = self.pb_ompl_interface.plan(pos_state)
        if res:
            if double_speed:
                del path[::2]
            self.solution.extend(path)
            self.pb_ompl_world.set_state(path[-1])

    def get_states(self, trajectory, puzzle_joint_index):
        states = []
        puzzle_joint_pos = 0

        for pos in trajectory:
            self.puzzle_state.change_joint_pos(puzzle_joint_index, puzzle_joint_pos)
            state, is_valid = self.get_state(pos)
            state = list(state)
            states.append(state)
            puzzle_joint_pos += self.config.step_size

        print(states[-1])
        self.solution.extend(states)
        self.pb_ompl_world.set_state(states[-1])
        return states

    def get_valid_state(self, pos):
        original_state = self.pb_ompl_world.get_cur_state()
        pos_state, is_valid = self.get_state(pos)
        if is_valid:
            self.pb_ompl_world.set_state(original_state)
            return pos_state, is_valid

        for i in range(8):
            for j in range(8):
                state = self.pb_ompl_world.get_cur_state()
                state[0] = i - 3.5
                state[1] = j - 3.5
                self.pb_ompl_world.set_state(state)
                pos_state, is_valid = self.get_state(pos)
                if is_valid:
                    self.pb_ompl_world.set_state(original_state)
                    return pos_state, is_valid

        self.pb_ompl_world.set_state(original_state)
        return pos_state, is_valid

    def get_state(self, pos):
        end_effector_index = 8
        orn = p.getQuaternionFromEuler((0, math.pi, 0))

        state = list(p.calculateInverseKinematics(self.pb_ompl_world.id, end_effector_index, pos, orn, maxNumIterations=300))

        for i in range(len(state)):
            state[i] = round(state[i], 5)  # otherwise some states might be slightly out of joint bounds

        temp_state = self.pb_ompl_world.get_cur_state()
        is_valid = self.pb_ompl_interface.is_state_valid(state)
        self.pb_ompl_world.set_state(temp_state)

        return state, is_valid

    def calculate_trajectory(self, start, end, step_size):
        trajectory = []

        while start != end:
            trajectory.append(start)
            diff = [e - s for e, s in zip(end, start)]
            offset = self.calculate_offset(diff, step_size)
            start = self.add_tuples(start, offset)

        trajectory.append(end)
        return trajectory

    def calculate_offset(self, diff, step_size):
        offset = []
        for value in diff:
            if -step_size <= value <= step_size:
                offset.append(value)
            elif value > step_size:
                offset.append(step_size)
            elif value < -step_size:
                offset.append(-step_size)

        return tuple(offset)

    def add_tuples(self, tuple_a: tuple, tuple_b: tuple) -> tuple:
        """Addition of two tuples."""
        return tuple(map(lambda x, y: x + y, tuple_a, tuple_b))
        # Source: https://stackoverflow.com/questions/497885/python-element-wise-tuple-operations-like-sum

    def execute(self, fps=200):
        for state in self.solution:
            self.pb_ompl_world.set_state(state)
            time.sleep(1. / fps)

        time.sleep(4.)
