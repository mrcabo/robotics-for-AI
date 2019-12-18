from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
import yaml
import os
class Navigation_test(AbstractBehaviour):
    """
    rosservice call /move_head "pitch: 0.6
    yaw: 0.0"

    rosservice call /move_base/clear_costmaps
    """
    def init(self):
        print(os.getcwd())
        self.navigation_sub = self.get_behaviour('NavigationSub')
        with open('/home/group9/catkin_ws/src/behaviours/scripts/behaviours/Navigation_test/simple_waypoints.yaml', 'r') as f:
            self.poses = yaml.load(f)
        self.waypoints = self.poses.keys()
        self.current_goal = 0

    def update(self):
        if self.state == State.start:
            print('Going to %s' % self.waypoints[self.current_goal])
            self.navigation_sub.start_nagivate_to(*self.get_pos_and_orientation_for_current_goal())
            self.set_state(State.navigating)
        elif self.state == State.navigating:
            if self.navigation_sub.finished():
                self.current_goal = (self.current_goal + 1) % len(self.waypoints)
                self.start()
            elif self.navigation_sub.failed():
                self.start()

    def get_pos_and_orientation_for_current_goal(self):
        pose_and_orientation = self.poses[self.waypoints[self.current_goal]]
        pos = self.create_pos_tuple(**pose_and_orientation['pos'])
        orientation = self.create_orientation_tuple(**pose_and_orientation['orientation'])
        return pos, orientation

    @staticmethod
    def create_pos_tuple(x, y):
        return x, y, 0

    @staticmethod
    def create_orientation_tuple(x, y, z, w):
        return x, y, z, w