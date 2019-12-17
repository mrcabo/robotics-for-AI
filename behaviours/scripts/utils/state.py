from enum import Enum, unique

@unique
class State(Enum):
    idle = 0
    start = 1
    finished = 2
    failed = 3
    waiting = 4
    approach_table = 5
    recognising = 6
    grasping = 7
    dropping = 8
    moving_backwards = 9

