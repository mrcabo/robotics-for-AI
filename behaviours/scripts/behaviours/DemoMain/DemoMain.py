import copy
import yaml

from rospy import ROSException
from std_msgs.msg import String
from my_msgs.msg import Order

from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class DemoMain(AbstractBehaviour):
    
    def init(self):
        self.grasp_main_sub = self.get_behaviour('GraspMainSub')
        self.drop_main_sub = self.get_behaviour('DropMainSub')
        self.navigation_sub = self.get_behaviour('NavigationSub')

        with open('/home/group9/catkin_ws/src/behaviours/scripts/behaviours/DemoMain/simulation_waypoints.yaml', 'r') as f:
            self.table_positions = yaml.load(f)

        self.target_tables = []
        self.target_items = []  # TODO: delete, this is just for debugging
        self.current_target = None
        self.current_destination = None
        self.box_in_hand = False
        self.text_to_speech_pub = rospy.Publisher('chatter', String, queue_size=3)

        self.text_to_speech_pub.publish("Ready to receive a new order")

    def start_navigating_to(self, destination):
        self.current_destination = destination
        print('Navigating to : {}'.format(self.target_tables[self.current_destination]))
        self.navigation_sub.start_nagivate_to(*self.get_pos_and_orientation_for_current_goal())

    def get_next_table_destination(self):
        if self.box_in_hand:
            return 0
        elif not self.current_destination or self.current_destination == 0:
            return 1
        else:
            if self.current_destination + 1 > len(self.target_tables):
                raise IndexError("No destinations left")
            return self.current_destination + 1

    def update(self):
        if self.state == State.start:
            try:
                order_msg = rospy.wait_for_message("/order", Order, .5)
                self.target_items = order_msg.objects
                self.target_tables = ['droptable'] + order_msg.tables
                print order_msg
                print('START: set state to select_table')
                self.set_state(State.select_table)
            except ROSException:
                pass

        elif self.state == State.select_table:
            if not self.target_items:
                print('No target items left, so finished!')
                self.finish()
            else:
                try:
                    self.start_navigating_to(self.get_next_table_destination())
                    self.set_state(State.navigating)
                except IndexError as e:
                    self.fail(e.message)

        elif self.state == State.navigating:
            if self.navigation_sub.finished():
                if self.box_in_hand:
                    self.drop_main_sub.start()
                    self.set_state(State.dropping)
                else:
                    self.grasp_main_sub.start_grasp(copy.deepcopy(self.target_items))
                    self.set_state(State.grasping)
            elif self.navigation_sub.failed():
                self.start_navigating_to(self.current_destination)
                self.set_state(State.navigating)

        elif self.state == State.grasping:
            if self.grasp_main_sub.finished():
                print("Grasp sub succeeded")
                self.box_in_hand = True
                self.set_state(State.select_table)
                print('Seen items during grasping: {}, grasped items: {}'.format(self.grasp_main_sub.get_seen_items(), self.grasp_main_sub.get_picked_up_item()))
            elif self.grasp_main_sub.failed():
                print("Grasp sub failed: {}".format(self.grasp_main_sub.failure_reason))
                print('Seen items during grasping: {}, grasped items: {}'.format(self.grasp_main_sub.get_seen_items(), self.grasp_main_sub.get_picked_up_item()))
                if self.grasp_main_sub.failure_reason == 'object_not_found':
                    print('set state to select_table')
                    self.set_state(State.select_table)
                else:
                    self.fail("TODO recovery behaviour: Grasping failed")

        elif self.state == State.dropping:
            if self.drop_main_sub.finished():
                self.box_in_hand = False
                self.set_state(State.select_table)
            elif self.drop_main_sub.failed():
                self.fail("TODO recovery behaviour: Dropping failed")

    def get_pos_and_orientation_for_current_goal(self):
        pose_and_orientation = self.table_positions[self.target_tables[self.current_destination]]
        pos = self.create_pos_tuple(**pose_and_orientation['pos'])
        orientation = self.create_orientation_tuple(**pose_and_orientation['orientation'])
        return pos, orientation

    @staticmethod
    def create_pos_tuple(x, y):
        return x, y, 0

    @staticmethod
    def create_orientation_tuple(x, y, z, w):
        return x, y, z, w