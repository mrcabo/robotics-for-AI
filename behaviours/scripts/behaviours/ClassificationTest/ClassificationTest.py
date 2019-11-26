from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class ClassificationTest(AbstractBehaviour):
    
    def init(self):
        self.classification_sub = self.get_behaviour('ClassificationSub')

    def update(self):
        if self.state == State.start:
            self.classification_sub.start()
            self.state = State.waiting
        elif self.state == State.waiting:
            if self.classification_sub.finished():
                self.finish()
            elif self.classification_sub.failed():
                self.start()
