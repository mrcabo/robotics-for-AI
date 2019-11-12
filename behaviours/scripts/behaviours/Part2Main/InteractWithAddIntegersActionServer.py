import random

import actionlib
from my_msgs.msg import AddIntegersAction, AddIntegersGoal

from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class InteractWithAddIntegersActionServer(AbstractBehaviour):

    def __init__(self, *args, **kwargs):
        super(InteractWithAddIntegersActionServer, self).__init__(*args, **kwargs)
        self.client = actionlib.SimpleActionClient('add_integers_server', AddIntegersAction)
    
    def init(self):
        print 'Connecting to server'
        self.client.wait_for_server()
        print 'Connected to the server'

    def update(self):
        # When the state is start, send two integers to the action server
        if self.state == State.start:
            goal = AddIntegersGoal()
            goal.lhs = random.randint(1, 10)
            goal.rhs = random.randint(1, 10)
            print "Sending two integers(%i, %i) to action server" % (goal.lhs, goal.rhs)
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
