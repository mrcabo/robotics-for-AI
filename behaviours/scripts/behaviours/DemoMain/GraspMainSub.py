from enum import Enum

from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class GraspMainSub(AbstractBehaviour):

    def init(self):
        self.grasp_sub = self.get_behaviour('GraspSub')
        self.approach_sub = self.get_behaviour('ApproachTableSub')
        self.move_back_sub = self.get_behaviour('MoveBackwardsSub')
        self.classification_sub = self.get_behaviour('ClassificationSub')

        self.successful_grasp = False

        self.matches = {}
        self.target_object_names = []

        self.current_match = None
        self.not_seen_objects = []

    def start_grasp(self, items):
        self.not_seen_objects = []
        self.matches = {}
        self.successful_grasp = False
        self.target_object_names = items
        self.state = State.start
        self.failure_reason = ''

    def start(self):
        raise NotImplementedError("Use start_grasp!")

    def remove_duplicate_matches(self, matches):
        """
        Dont put two of the same boxes!!!
        """
        filtered = {}
        for match in matches:
            if match.class_name in filtered:
                if match.center_z > filtered[match.class_name].center_z:
                    filtered[match.class_name] = match
            else:
                filtered[match.class_name] = match

        return filtered

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
                # filter out matches that are in the target_objects_names
                matches_with_duplicates = list(filter(lambda o: o.class_name in self.target_object_names, self.classification_sub.result))
                # remove duplicate matches while choosing the highest z
                self.matches = self.remove_duplicate_matches(matches_with_duplicates)
                # filter the target object names on the matches we found
                self.target_object_names = list(filter(lambda n: n in self.matches.keys(), self.target_object_names))

                if not self.matches:
                    self.failure_reason = 'object_not_found'
                    print("object_not_found")
                    self.set_state(State.moving_backwards)
                    self.move_back_sub.start()
                elif not self.target_object_names:
                    self.failure_reason = "tried_all_targets"
                    print("no more targets in target object names left")
                    self.set_state(State.moving_backwards)
                    self.move_back_sub.start()
                else:
                    self.set_state(State.grasping)
                    self.current_match = self.target_object_names.pop(0)
                    self.grasp_sub.start_pickup(self.matches[self.current_match])

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
                print("Start moving backwards sub behaviour")
                self.successful_grasp = True
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()
            elif self.grasp_sub.failed():
                print("Grasping sub behavior failed with reason: %s" % self.grasp_sub.failure_reason)
                print("Start moving backwards sub behaviour")
                if self.grasp_sub.failure_reason == 'no_matching_bounding_box':
                    self.not_seen_objects.append(self.current_match)
                self.successful_grasp = False
                self.set_state(State.moving_backwards)
                self.move_back_sub.start()

        # Also ping the moving backwards sub
        # If move backwards sub succeeds -> if not dropped the box yet approach table
        # Fails -> for now kill this program
        elif self.state == State.moving_backwards:
            if self.move_back_sub.finished():
                if self.failure_reason:
                    self.fail(self.failure_reason)
                elif self.successful_grasp:
                    print("The main behaviour succeeded:P!")
                    self.finish()
                else:
                    print("approaching table for new grasp try")
                    self.approach_sub.start_approach(0.45)
                    self.set_state(State.approach_table)
            elif self.move_back_sub.failed():
                print("Move back sub behavior failed with reason: %s" % self.move_back_sub.failure_reason)
                print("Main behaviour fails")
                self.fail("Moving backwards failed")

    def get_seen_items(self):
        return [match.class_name for key, match in self.matches.items() if key not in self.not_seen_objects]

    def get_picked_up_item(self):
        return self.current_match if self.successful_grasp else None
