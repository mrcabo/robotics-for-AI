from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class GraspTestMain(AbstractBehaviour):
    
    def init(self):
        self.grasp_sub = self.get_behaviour('GraspSub')
        self.approach_sub = self.get_behaviour('ApproachTableSub')
        self.droping_sub = self.get_behaviour('DropSub')

    def update(self):
        if self.state == State.start:
            # Do final approximation to the table
            print("Approaching table")
            self.approach_sub.start_approach(0.35)
            self.set_state(State.grasp_approach_table)

        elif self.state == State.grasp_approach_table:
            if self.approach_sub.finished():
                print("Starting grasping sub behaviour")
                self.set_state(State.grasping)
                self.grasp_sub.start()
            elif self.approach_sub.failed():
                print("Final approximation to the table failed")
                # TODO: manage this somehow? move backward a bit and try again?
                self.start()

        elif self.state == State.grasping:
            if self.grasp_sub.finished():
                print("Approaching table")
                self.approach_sub.start_approach(0.35)
                self.set_state(State.drop_approach_table)
            elif self.grasp_sub.failed():
                self.start()

        elif self.state == State.drop_approach_table:
            if self.approach_sub.finished():
                print("Starting dropping sub behaviour")
                self.set_state(State.droping)
                self.droping_sub.start()
            elif self.approach_sub.failed():
                print("Final approximation to the table failed")
                # TODO: manage this somehow? move backward a bit and try again?
                self.start()

        elif self.state == State.droping:
            if self.droping_sub.finished():
                self.finish()
            elif self.droping_sub.failed():
                self.start()

