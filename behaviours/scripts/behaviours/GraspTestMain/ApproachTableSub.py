import actionlib
from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
from approach.msg import ApproachTableAction, ApproachTableGoal


class ApproachTableSub(AbstractBehaviour):

    def init(self):
        self.client = actionlib.SimpleActionClient("/approach_table", ApproachTableAction)
        print 'Connecting to approach_table server'
        self.client.wait_for_server()
        print 'Connected to the approach_table server'
        self.distance_to_table = None

    def update(self):
        # When the state is start, send two integers to the action server
        if self.state == State.start:
            goal_msg = ApproachTableGoal()
            goal_msg.transform_to_link = "base_link"
            goal_msg.min_z = 0.4
            goal_msg.max_z = 1.5
            goal_msg.distance_to_table = self.distance_to_table  # For tiago it should be 0.35
            self.client.send_goal(goal_msg)

            self.state = State.waiting
        # When the state is waiting, ping the action server to see if it succeeded already
        if self.state == State.waiting:
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                self.finish()
            elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
                self.fail('/approach_table action server failed')

    def start_approach(self, distance):
        self.distance_to_table = distance
        self.set_state(State.start)

    def start(self):
        raise NotImplementedError("Use start navigate to!")

    def reset(self):
        self.state = State.idle
        self.init()
