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

        with open('/home/pal/catkin_ws/src/group9/behaviours/scripts/behaviours/DemoMain/real_waypoints.yaml', 'r') as f:
            self.table_positions = yaml.load(f)

        self.target_tables = []
        self.target_items = []  # TODO: delete, this is just for debugging
        self.table_observation = []
        self.current_target = None
        self.current_destination = None
        self.box_in_hand = False
        self.text_to_speech_pub = rospy.Publisher('/speech', String, queue_size=3)
        self.dropped_items = []
        self.item_in_gripper = None
        
        rospy.sleep(2)
        self.text_to_speech_pub.publish("Ready to receive a new order")

    def start_navigating_to(self, destination):
        self.current_destination = destination
        self.text_to_speech_pub.publish('Navigating to {}'.format(self.target_tables[self.current_destination]))
        print('Navigating to : {}'.format(self.target_tables[self.current_destination]))
        self.navigation_sub.start_nagivate_to(*self.get_pos_and_orientation_for_current_goal())

    def target_can_be_in_destination(self, destination):
        destination_observation = self.table_observation[destination]
        if destination_observation is None:
            return True
        intersection = set(destination_observation).intersection(set(self.target_items))
        print("Intersection of {} and {} is {}".format(destination_observation, self.target_items, intersection))
        return bool(intersection)

    def get_next_table_destination(self):
        print("current memory = {}".format(self.table_observation))
        if not self.target_items:
            raise IndexError("No target items left")
        if self.box_in_hand:
            return 0
        elif self.current_destination is None or (self.current_destination == 0 and self.target_can_be_in_destination(1)):
            return 1
        else:
            if self.current_destination + 1 > len(self.target_tables):
                raise IndexError("No destinations left")
            if self.target_can_be_in_destination(2):
                return 2
            else:
                raise IndexError("Cant find all target boxes")

    def update(self):
        if self.state == State.start:
            try:
                order_msg = rospy.wait_for_message("/order", Order, .5)
                self.target_items = order_msg.objects
                self.target_tables = ['drop table'] + order_msg.tables
                self.table_observation = [[]] + [None] * len(order_msg.tables)
                self.current_target = None
                self.current_destination = None
                self.box_in_hand = False
                self.grasped_items = []
                print order_msg
                print('START: set state to select_table')
                self.set_state(State.select_table)
            except ROSException:
                pass

        elif self.state == State.select_table:
            try:
                next_table_idx = self.get_next_table_destination()
                if next_table_idx != 0:
                    self.text_to_speech_pub.publish("I am going to find {} on {}!"
                        .format(" ".join(self.target_items), self.target_tables[next_table_idx]))
                self.start_navigating_to(next_table_idx)
                self.set_state(State.navigating)
            except IndexError as e:
                print("Going to home because: {}".format(e.message))
                self.text_to_speech_pub.publish('Navigating to start position')
                pose_and_orientation = self.table_positions['home']
                pos = self.create_pos_tuple(**pose_and_orientation['pos'])
                orientation = self.create_orientation_tuple(**pose_and_orientation['orientation'])
                self.navigation_sub.start_nagivate_to(pos, orientation)
                self.set_state(State.return_home)

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
                grasped_item = self.grasp_main_sub.get_picked_up_item()
                seen_items = self.grasp_main_sub.get_seen_items()
                if grasped_item in seen_items:
                    seen_items.remove(grasped_item)
                self.item_in_gripper = grasped_item
                self.table_observation[self.current_destination] = seen_items
                print('Seen items during grasping: {}, grasped items: {}'.format(seen_items, grasped_item))
                self.box_in_hand = True
                self.set_state(State.select_table)
            elif self.grasp_main_sub.failed():
                print("Grasp sub failed: {}".format(self.grasp_main_sub.failure_reason))
                grasped_item = self.grasp_main_sub.get_picked_up_item()
                seen_items = self.grasp_main_sub.get_seen_items()
                self.table_observation[self.current_destination] = seen_items
                print('Seen items during grasping: {}, grasped items: {}'.format(seen_items, grasped_item))
                if self.grasp_main_sub.failure_reason == 'object_not_found':
                    print('set state to select_table')
                    self.set_state(State.select_table)
                else:
                    self.fail("TODO recovery behaviour: Grasping failed")

        elif self.state == State.dropping:
            if self.drop_main_sub.finished():
                self.box_in_hand = False
                if self.item_in_gripper in self.target_items:
                    self.target_items.remove(self.item_in_gripper)
                    self.dropped_items.append(self.item_in_gripper)
                self.item_in_gripper = None
                self.set_state(State.select_table)
            elif self.drop_main_sub.failed():
                self.fail("TODO recovery behaviour: Dropping failed")

        elif self.state == State.return_home:
            if self.navigation_sub.finished():
                self.text_to_speech_pub.publish("Order done, we dropped of {}".format(" ".join(self.dropped_items)))
                self.text_to_speech_pub.publish("Ready to receive a new order")
                self.set_state(State.start)
            elif self.navigation_sub.failed():
                self.text_to_speech_pub.publish("Order done, we dropped of {}".format(" ".join(self.dropped_items)))
                self.text_to_speech_pub.publish("Ready to receive a new order")
                self.set_state(State.start)

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