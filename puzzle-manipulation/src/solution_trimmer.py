MAX_JOINT_MOVEMENT_PER_FRAME = 0.002


def trim(solution: list[list, ...]):
    i = 0
    while i + 1 < len(solution):
        check_following(i, solution)
        i += 1


def check_following(i: int, solution: list[list, ...]):
    while i + 1 < len(solution):
        if minor_diff(solution[i], solution[i + 1]):
            del solution[i + 1]
        else:
            return


def minor_diff(state1: list, state2: list):
    for i, _ in enumerate(state1):
        if abs(state1[i] - state2[i]) > MAX_JOINT_MOVEMENT_PER_FRAME:
            return False
    return True
