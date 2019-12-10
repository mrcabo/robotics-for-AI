from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class GraspTestMain(AbstractBehaviour):
    
    def init(self):
        self.grasp_sub = self.get_behaviour('GraspSub')
        self.approach_sub = self.get_behaviour('ApproachTableSub')
        self.move_back_sub = self.get_behaviour('MoveBackwardsSub')
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
                print("Final approximation for grasping to the table failed")
                # TODO: manage this somehow? move backward a bit and try again?
                self.start()

        elif self.state == State.grasping:
            if self.grasp_sub.finished():
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()
            elif self.grasp_sub.failed():
                print("Grasping sub behavior failed with reason: %s" % self.grasp_sub.failure_reason)
                self.start()

        elif self.state == State.moving_backwards:
            if self.move_back_sub.finished():
                print("Approaching table")
                self.approach_sub.start_approach(0.35)
                self.set_state(State.drop_approach_table)
            elif self.move_back_sub.failed():
                print("Move back sub behavior failed with reason: %s" % self.move_back_sub.failure_reason)
                self.start()

        elif self.state == State.drop_approach_table:
            if self.approach_sub.finished():
                print("Starting dropping sub behaviour")
                self.set_state(State.droping)
                self.droping_sub.start()
            elif self.approach_sub.failed():
                print("Final approximation for dropping to the table failed")
                # TODO: manage this somehow? move backward a bit and try again?
                self.start()

        elif self.state == State.droping:
            if self.droping_sub.finished():
                print("Drop sub finished succesfully")
                self.finish()
            elif self.droping_sub.failed():
                print("Drop sub failed with: %s" % self.droping_sub.failure_reason)
                self.start()

