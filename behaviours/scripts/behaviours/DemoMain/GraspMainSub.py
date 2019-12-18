from enum import Enum

from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class GraspMainSub(AbstractBehaviour):

    def init(self):
        self.grasp_sub = self.get_behaviour('GraspSub')
        self.approach_sub = self.get_behaviour('ApproachTableSub')
        self.move_back_sub = self.get_behaviour('MoveBackwardsSub')
        self.dropping_sub = self.get_behaviour('DropSub')
        self.classification_sub = self.get_behaviour('ClassificationSub')

        self.box_in_hand = False
        self.dropped_the_box = False

        self.recognised_objects = None
        self.target_object_name = ''

    def start_grasp(self, item):
        self.target_object_name = item
        self.state = State.start
        self.failure_reason = ''

    def start(self):
        raise NotImplementedError("Use start pickup!")

    def update(self):
        if self.state == State.start:
            # Do final approximation to the table
            print("START: Approaching table")
            self.approach_sub.start_approach(0.45)
            self.set_state(State.approach_table)

        # If in approach_table state we ping the subbehaviour
        # if approach sub succeeds ->  go to grasp or drop according to whether we have a box in hand
        # if approach sub fails -> go to moving backwards
        elif self.state == State.approach_table:
            if self.approach_sub.finished():
                if self.box_in_hand:
                    print("Starting dropping sub behaviour")
                    self.set_state(State.dropping)
                    self.dropping_sub.start()
                else:
                    print("Starting recognising sub behaviour")
                    self.set_state(State.recognising)
                    self.recognised_objects = None
                    self.classification_sub.start()
            elif self.approach_sub.failed():
                print("Final approximation to the table failed")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()

        # When recognising ping classification sub
        # If classification sub succeeds -> go to grasping sub
        # If classification sub fails -> moving backwards
        elif self.state == State.recognising:
            if self.classification_sub.finished():
                print("Starting grasping sub behaviour")
                self.recognised_objects = self.classification_sub.result  # TODO do we need it to be class variable?
                matches = [x for x in self.recognised_objects if (x.class_name == self.target_object_name)]
                if len(matches) == 1:
                    target = matches[0]
                    self.set_state(State.grasping)
                    self.grasp_sub.start_pickup(target)
                elif len(matches) > 1:
                    target = sorted(matches, key=lambda x: x.center_z, reverse=True)
                    self.set_state(State.grasping)
                    self.grasp_sub.start_pickup(target)
                else:
                    self.failure_reason = 'object_not_found'
                    print("object_not_found")
                    self.set_state(State.moving_backwards)
                    self.move_back_sub.start()

            elif self.classification_sub.failed():
                print("Classification sub behavior failed with reason: %s" % self.classification_sub.failure_reason)
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()

        # When grasping ping grasp sub
        # If grasp sub succeeds -> set box in hand True and moving backwards
        # If grasp sub fails -> moving backwards
        elif self.state == State.grasping:
            if self.grasp_sub.finished():
                self.box_in_hand = True
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()
            elif self.grasp_sub.failed():
                self.box_in_hand = False
                print("Grasping sub behavior failed with reason: %s" % self.grasp_sub.failure_reason)
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()

        # Also ping the moving backwards sub
        # If move backwards sub succeeds -> if not dropped the box yet approach table
        # Fails -> for now kill this program
        elif self.state == State.moving_backwards:
            if self.move_back_sub.finished():
                if self.failure_reason:
                    self.fail(self.failure_reason)
                elif self.dropped_the_box:
                    print("The main behaviour succeeded:P!")
                    self.finish()
                else:
                    print("Approaching table")
                    self.approach_sub.start_approach(0.45)
                    self.set_state(State.approach_table)
            elif self.move_back_sub.failed():
                print("Move back sub behavior failed with reason: %s" % self.move_back_sub.failure_reason)
                print("Main behaviour fails")
                self.fail("Moving backwards failed")

        # Yeye ping the behaviour
        # if drop sub succeeds -> set dropped_the_box to True and move backwards
        # if drop sub fails -> move backwards
        elif self.state == State.dropping:
            if self.dropping_sub.finished():
                print("Drop sub finished succesfully")
                self.dropped_the_box = True
                self.box_in_hand = False
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()
            elif self.dropping_sub.failed():
                print("Drop sub failed with: %s" % self.dropping_sub.failure_reason)
                print("Start moving backwards sub behaviour")
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()
