import time

from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class Sub2Wait(AbstractBehaviour):
    
    def init(self):
        self.second_at_init = time.time()

    def update(self):
        if time.time() - self.second_at_init >= 2:
            self.finish()
    
    def reset(self):
        self.state = State.idle
        self.init()
