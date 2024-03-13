import os
import random
import sys
import time

import pybullet as p
import pybullet_data

SEED = 0
RANDOM_START_STATES_FOR_IK_NUM = 100000
GRIP_ACTION_HEIGHT = 0.5

dirname = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dirname)

import pb_ompl
from puzzle import Puzzle
from config.configuration import Action, Configuration
from inverse_kinematics import InverseKinematics
from solution_trimmer import trim, minor_diff


class Manipulation:
    def __init__(self, config: Configuration):
        self.config = config
        self.solution = []
        self.obstacles = []

        self.set_up_pybullet_environment()

        plane_id = p.loadURDF("plane.urdf")
        self.obstacles.append(plane_id)

        puzzle_id = self.load(config.puzzle_urdf)
        self.obstacles.append((puzzle_id, frozenset(list(range(p.getNumJoints(puzzle_id))))))
        self.puzzle = Puzzle(puzzle_id, self.config)

        robot_id = self.load(config.robot_urdf, (0, 0, 0.005), scale=4)
        self.robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot.set_state(list(config.robot_start_state))

        print("number of dimensions (robot) = " + str(self.robot.num_dim))
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)
        self.pb_ompl_interface.set_planner(config.ompl_planner)

        self.ik = InverseKinematics(self.robot.id)

        random.seed(SEED)

    def set_up_pybullet_environment(self):
        if p.isConnected():
            p.resetSimulation()
        else:
            p.connect(p.GUI)
            p.setGravity(0, 0, -9.8)
            p.setTimeStep(1. / 240.)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            self.position_camera()

    def position_camera(self):
        camera_info = p.getDebugVisualizerCamera()
        print('camera info: ' + str(camera_info))
        yaw = camera_info[8]
        pitch = camera_info[9]
        dist = camera_info[10]
        target_pos = camera_info[11]
        pitch = -55
        dist = 6
        target_pos = (2, -2, 0)
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target_pos)
        camera_info = p.getDebugVisualizerCamera()
        print('camera info: ' + str(camera_info))

    def load(self, urdf: str, location=(0, 0, 0), rotation=(0, 0, 0), scale=1):
        orn = p.getQuaternionFromEuler(rotation)
        flags = p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        pb_id = p.loadURDF(urdf, location, orn, useFixedBase=1, flags=flags, globalScaling=scale)

        print("\ninfo for " + urdf + ":")
        self.print_joints(pb_id)

        return pb_id

    def print_joints(self, id):
        print("\nnumber of joints = " + str(p.getNumJoints(id)))
        for i in range(p.getNumJoints(id)):
            print(p.getJointInfo(id, i))
        print("\n")

    def plan(self):
        # self.debug()

        for action in self.config.action_sequence:
            self.plan_action(action)

        self.plan_and_move_to_state(self.config.robot_final_state)

    def debug(self):
        self.robot.set_state([-0.02671, 0.42166, -0.2728, -0.13667, -0.60573, -2.36092, -0.09954, 0.89569, -0.81182])
        self.puzzle.set_state([0.28, 0.24, -1.56, 0.24])
        time.sleep(60 * 10)

    def plan_action(self, action: Action):
        self.grab(action)
        self.actuate(action)

    def grab(self, action):
        target_pos = self.get_target_pos(action.grip_point)
        target_pos_high = self.add_tuples(target_pos, (0, 0, GRIP_ACTION_HEIGHT))

        self.plan_and_move_to_pos(target_pos_high)
        self.plan_and_move_to_pos(target_pos)

    def release(self, action):
        target_pos = self.get_target_pos(action.grip_point)
        target_pos_high = self.add_tuples(target_pos, (0, 0, GRIP_ACTION_HEIGHT))
        self.plan_and_move_to_pos(target_pos_high)

    def get_target_pos(self, target_name: str):
        joint_index = self.puzzle.joint_index_map[target_name]
        world_pos = p.getLinkState(self.puzzle.id, joint_index)[0]
        shape_data = p.getCollisionShapeData(self.puzzle.id, joint_index)[0]
        scale = shape_data[3]
        pos = self.add_tuples(world_pos, (0, 0, scale[2]))
        return pos

    def actuate(self, action: Action):
        diff = self.target_diff(action)
        while abs(diff) > self.config.step_size:

            start_pos = self.get_target_pos(action.grip_point)
            current_start_state = self.robot.get_cur_state()

            self.move_puzzle_forward(action, diff)
            if not self.is_valid(current_start_state):

                self.move_puzzle_backward(action, diff)
                new_start_state = self.get_valid_state(start_pos, action, diff)
                self.interpolate_or_plan(new_start_state)
                self.move_puzzle_forward(action, diff)

            goal_pos = self.get_target_pos(action.grip_point)
            goal_state = self.get_valid_state(goal_pos)

            self.move_puzzle_backward(action, diff)
            if not self.is_valid(goal_state):

                goal_state = self.get_valid_state(goal_pos, action, diff)

            self.interpolate_or_plan(goal_state)

            self.move_puzzle_forward(action, diff)

            diff = self.target_diff(action)

    def interpolate_or_plan(self, goal_state):
        # if self.minor_diff(goal_state):  # TODO
        #     self.interpolate(goal_state)
        #     return

        self.plan_and_move_to_state_fast(goal_state)

    def plan_and_move_to_state_fast(self, state):
        start_state = self.robot.get_cur_state()

        success, path = self.pb_ompl_interface.plan_fast(state)

        if success and minor_diff(start_state, path[-1], 0.1):
            self.solution_append(path)
        else:
            self.robot.set_state(start_state)
            self.plan_and_move_to_state(state)

    def plan_and_move_to_state(self, state):
        self.print_planning_info(state)

        success, path = self.pb_ompl_interface.plan(state)

        if success:
            self.solution_append(path)
        else:
            raise Exception("Planning failed")

    def print_planning_info(self, state):
        print("-" * 100)
        print("Planning")
        print("start: " + str(self.robot.get_cur_state()))
        print("goal:  " + str(state))
        print("-" * 100)

    def solution_append(self, path):
        trim(path)
        self.solution.extend(path)
        self.robot.set_state(path[-1])
        self.puzzle.add_current_state_to_solution(len(path))

    def move_puzzle_forward(self, action, diff):
        if diff < 0:
            self.puzzle.move_joint(action.joint_index, -self.config.step_size)
        else:
            self.puzzle.move_joint(action.joint_index, self.config.step_size)

    def move_puzzle_backward(self, action, diff):
        if diff < 0:
            self.puzzle.move_joint(action.joint_index, self.config.step_size)
        else:
            self.puzzle.move_joint(action.joint_index, -self.config.step_size)

    def target_diff(self, action):
        target = action.joint_pos
        actual = self.puzzle.get_joint_pos(action.joint_index)
        return target - actual

    def plan_and_move_to_pos(self, pos):
        state = self.get_valid_state(pos)
        self.plan_and_move_to_state(state)

    def get_valid_state(self, pos, action=None, diff=None):
        state, is_valid = self.get_state_for_moving_puzzle(pos, action, diff)
        if is_valid:
            return state

        original_state = self.robot.get_cur_state()
        for i in range(RANDOM_START_STATES_FOR_IK_NUM):
            start_state = self.get_random_state()
            self.robot.set_state(start_state)
            state, is_valid = self.get_state_for_moving_puzzle(pos, action, diff)
            if is_valid:
                self.robot.set_state(original_state)
                return state

        self.robot.set_state(original_state)
        raise Exception("Could not sample valid state")

    def get_random_state(self):
        m = map(self.random_num, self.robot.joint_bounds)
        return list(m)

    def random_num(self, limits):
        r = random.uniform(limits[0], limits[1])
        return round(r, 5)

    def get_state_for_moving_puzzle(self, pos, action, diff):
        state, is_valid = self.get_state(pos)

        if action is None:
            return state, is_valid

        self.move_puzzle_forward(action, diff)
        is_still_valid = self.is_valid(state)
        self.move_puzzle_backward(action, diff)

        return state, is_valid and is_still_valid

    def get_state(self, pos):
        state = self.ik.solve(pos)
        is_valid = self.is_valid(state)
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

    def is_valid(self, state):
        temp_state = self.robot.get_cur_state()
        no_collision = self.pb_ompl_interface.is_state_valid(state)
        within_joint_bounds = self.is_within_joint_bounds(state)
        self.robot.set_state(temp_state)
        return no_collision and within_joint_bounds

    def is_within_joint_bounds(self, state):
        for i in range(len(state)):
            lower = self.robot.joint_bounds[i][0]
            upper = self.robot.joint_bounds[i][1]
            if state[i] < lower or state[i] > upper:
                return False
        return True

    def execute(self, fps=60):
        for robot_state, puzzle_state in zip(self.solution, self.puzzle.solution):
            self.robot.set_state(robot_state)
            self.puzzle.set_state(puzzle_state)
            time.sleep(1. / fps)

        time.sleep(4)
