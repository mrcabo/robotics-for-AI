from enum import Enum, unique

@unique
class State(Enum):
    idle = 0
    start = 1
    finished = 2
    failed = 3
    waiting = 4
    approach_table = 5
    grasping = 6
    dropping = 7
    moving_backwards = 8

