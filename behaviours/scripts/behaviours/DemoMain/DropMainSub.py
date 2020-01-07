from enum import Enum

from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class DropMainSub(AbstractBehaviour):

    def init(self):
        self.approach_sub = self.get_behaviour('ApproachTableSub')
        self.move_back_sub = self.get_behaviour('MoveBackwardsSub')
        self.dropping_sub = self.get_behaviour('DropSub')

        self.dropped_the_box = False

    def update(self):
        if self.state == State.start:
            # Do final approximation to the table
            print("START: Approaching table")
            self.approach_sub.start_approach(0.45)
            self.set_state(State.approach_table)
            self.dropped_the_box = False

        # If in approach_table state we ping the subbehaviour
        # if approach sub succeeds ->  go to grasp or drop according to whether we have a box in hand
        # if approach sub fails -> go to moving backwards
        elif self.state == State.approach_table:
            if self.approach_sub.finished():
                print("Starting dropping sub behaviour")
                self.set_state(State.dropping)
                self.dropping_sub.start()
            elif self.approach_sub.failed():
                print("Final approximation to the table failed")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()

        # Also ping the moving backwards sub
        # If move backwards sub succeeds -> if not dropped the box yet approach table
        # Fails -> for now kill this program
        elif self.state == State.moving_backwards:
            if self.move_back_sub.finished():
                if self.dropped_the_box:
                    print("The main drop behaviour succeeded:P!")
                    self.finish()
                else:
                    print("Approaching table")
                    self.approach_sub.start_approach(0.45)
                    self.set_state(State.approach_table)
            elif self.move_back_sub.failed():
                print("Move back sub behavior failed with reason: %s" % self.move_back_sub.failure_reason)
                print("Main drop behaviour fails")
                self.fail("Moving backwards failed")

        # When state is dropping ping the behaviour to see if it finished
        # if drop sub succeeds -> set dropped_the_box to True and move backwards
        # if drop sub fails -> move backwards
        elif self.state == State.dropping:
            if self.dropping_sub.finished():
                print("Drop sub finished succesfully")
                self.dropped_the_box = True
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()
            elif self.dropping_sub.failed():
                print("Drop sub failed with: %s" % self.dropping_sub.failure_reason)
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()
