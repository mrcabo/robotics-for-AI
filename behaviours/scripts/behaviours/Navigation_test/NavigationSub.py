import time

import actionlib

from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy


class NavigationSub(AbstractBehaviour):
    
    def init(self):
        # Connect to the action server
        if not hasattr(self, 'client'):
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print 'Connecting to server'
        self.client.wait_for_server()
        print 'Connected to the server'

        self.pos = None
        self.orientation = None

    def update(self):
        # When mode is start build a message and send it to the action server
        if self.state == State.start:
            goal = self.build_goal_message()
            time.sleep(2)
            self.client.send_goal(goal)
            time.sleep(2)
            self.client.send_goal(goal)
            self.set_state(State.navigating)
        # While navigating, keep polling the action server for statuses
        elif self.state == State.navigating:
            client_state = self.client.get_state()
            if client_state == actionlib.GoalStatus.SUCCEEDED:
                print('Navigation completed')
                self.finish()
            elif client_state == actionlib.GoalStatus.ABORTED:
                print('Navigation failed')
                self.fail(reason="Action server returned ABORTED status")

    def reset(self):
        self.state = State.idle
        self.init()

    def start_nagivate_to(self, pos, orientation):
        self.pos = pos
        self.orientation = orientation
        self.state = State.start

    def start(self):
        raise NotImplementedError("Use start navigate to!")

    def build_goal_message(self):
        goal = MoveBaseGoal()
        # Set Header
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Set the Pose
        goal.target_pose.pose.position.x = self.pos[0]
        goal.target_pose.pose.position.y = self.pos[1]
        goal.target_pose.pose.position.z = self.pos[2]
        goal.target_pose.pose.orientation.x = self.orientation[0]
        goal.target_pose.pose.orientation.y = self.orientation[1]
        goal.target_pose.pose.orientation.z = self.orientation[2]
        goal.target_pose.pose.orientation.w = self.orientation[3]

        return goal