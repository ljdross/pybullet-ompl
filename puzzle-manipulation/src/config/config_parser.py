import json5
import os
import sys

dirname = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dirname)

from configuration import Action, Configuration


class ConfigParser:
    def __init__(self, config_json: str, defaults_json: str):
        with open(defaults_json) as f1:
            self.defaults = json5.load(f1)

        with open(config_json) as f2:
            self.config = json5.load(f2)

    def parse(self) -> Configuration:
        return Configuration(
            robot_urdf=self.get('robot_urdf'),
            robot_scale=self.get('robot_scale'),
            robot_start_state=self.get('robot_start_state'),
            robot_final_state=self.get('robot_final_state'),
            robot_x_limits=self.get('robot_x_limits'),
            robot_y_limits=self.get('robot_y_limits'),

            puzzle_urdf=self.get('puzzle_urdf'),
            puzzle_start_state=self.get('puzzle_start_state'),

            action_sequence=self.get('action_sequence'),
            grab_action_height=self.get('grab_action_height'),
            grab_gap=self.get('grab_gap'),
            ompl_planner=self.get('ompl_planner'),
            planning_time=self.get('planning_time'),
            step_size=self.get('step_size'),
        )

    def get(self, key: str):
        value = self.get_value(key)

        if key == "action_sequence":
            return self.deserialize_action_sequence(value)

        return value

    def get_value(self, key: str):
        if key in self.config:
            return self.config[key]
        elif key in self.defaults:
            return self.defaults[key]
        else:
            raise Exception('Key: "' + key + '" neither found in config, nor in defaults!')

    def deserialize_action_sequence(self, value):
        sequence = []
        for action in value:
            sequence.append(Action(action[0], action[1], action[2]))
        return sequence

