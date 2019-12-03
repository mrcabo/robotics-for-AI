import actionlib
from my_msgs.msg import GraspAction, GraspGoal

from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class GraspSub(AbstractBehaviour):

    def init(self):
        self.client = actionlib.SimpleActionClient('grasp_action_server', GraspAction)
        print 'Connecting to server'
        self.client.wait_for_server()
        print 'Connected to the server'

    def update(self):
        # When the state is start, send two integers to the action server
        if self.state == State.start:
            goal = GraspGoal()
            goal.s = "whatever"
            print "Sending %s to action server" % goal.s
            self.client.send_goal(goal)
            self.state = State.waiting
        # When the state is waiting, ping the action server to see if it succeeded already
        if self.state == State.waiting:
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                result = self.client.get_result()
                print 'Action server succeeded with result: ', result.value
                self.finish()
            elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
                result = self.client.get_result()
                print 'Action server failed with result: ', result.value
                self.fail('Action server failed')

    def reset(self):
        self.state = State.idle
        self.init()