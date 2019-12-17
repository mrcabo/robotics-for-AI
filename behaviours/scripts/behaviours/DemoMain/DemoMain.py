from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class DemoMain(AbstractBehaviour):
    
    def init(self):
        self.grasp_main_sub = self.get_behaviour('GraspMainSub')

    def update(self):
        if self.state == State.start:
            # Do final approximation to the table
            print("START: Grasping")
            self.grasp_main_sub.start()
            self.set_state(State.grasping)

        elif self.state == State.grasping:
            if self.grasp_main_sub.finished():
                self.finish()
            elif self.grasp_main_sub.failed():
                self.fail("Grasping failed")
