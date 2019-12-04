from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class GraspTestMain(AbstractBehaviour):
    
    def init(self):
        self.grasp_sub = self.get_behaviour('GraspSub')


    def update(self):
        # When in starting state, start the behaviour that will talk to the ActionServer
        if self.state == State.start:
            self.grasp_sub.start()
            self.state = State.waiting
        # When the state is waiting check the whether the sub behaviour finished or failed
        elif self.state == State.waiting:
            if self.grasp_sub.finished():
                self.finish()
            elif self.grasp_sub.failed():
                self.start()
