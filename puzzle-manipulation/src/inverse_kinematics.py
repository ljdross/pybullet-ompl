import math
import pybullet as p


class InverseKinematics:
    def __init__(self, pb_id: int):
        self.pb_id = pb_id

        # the following is for defining joint limits
        # unfortunately, it does not work well
        # see also: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?pli=1#heading=h.9i02ojf4k3ve
        # # lower limits for null space
        # self.ll = [-4, -4, -2.9670, -2.0943, -2.9670, -2.0943, -2.9670, -2.0943, -3.0543]
        # # upper limits for null space
        # self.ul = [4, 4, 2.9670, 2.0943, 2.9670, 2.0943, 2.9670, 2.0943, 3.0543]
        # # joint ranges for null space
        # self.jr = [7, 7, 5, 4, 5, 4, 5, 4, 6]
        # # restposes for null space
        # self.rp = [-1, -1, 0, 0, 0, -1, 0, 1.5, 0]

        self.end_effector_index = 8
        self.orn = p.getQuaternionFromEuler((0, math.pi, 0))
        self.iterations = 300

    def solve(self, pos):
        state = p.calculateInverseKinematics(self.pb_id, self.end_effector_index, pos, self.orn, maxNumIterations=self.iterations)
        state = list(state)

        for i in range(len(state)):
            state[i] = round(state[i], 5)  # otherwise some states might be slightly out of joint bounds

        return state
