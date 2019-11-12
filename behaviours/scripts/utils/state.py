from enum import Enum, unique

@unique
class State(Enum):
    idle = 0
    start = 1
    finished = 2
    failed = 3
    selecting_name = 4
    waiting = 5
    printing = 6
