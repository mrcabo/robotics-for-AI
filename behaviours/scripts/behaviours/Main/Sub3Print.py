import random

from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class Sub3Print(AbstractBehaviour):
    
    def init(self):
        self.name = "UNSET"

    def update(self):
        if random.random() > 0.5:
            print self.name
            self.finish()
        else:
            self.fail('value smaller than 0.5')

    def reset(self):
        self.state = State.idle
        self.init()

    def set_name(self, name):
        self.name = name