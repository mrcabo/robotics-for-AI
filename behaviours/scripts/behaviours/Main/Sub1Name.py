import random

from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class Sub1Name(AbstractBehaviour):

    def init(self):
        self.name = random.choice(['TIAGO', 'ALICE'])

    def update(self):
        self.finish()

    def reset(self):
        self.state = State.idle
        self.init()
