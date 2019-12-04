from enum import Enum, unique

@unique
class State(Enum):
    idle = 0
    start = 1
    finished = 2
    failed = 3
    waiting = 4
    grasp_approach_table = 5
    grasping = 6
    drop_approach_table = 7
    droping = 8

