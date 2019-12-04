import actionlib
from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
from approach.msg import ApproachTableAction


class ApproachTableSub(AbstractBehaviour):

    def init(self):
        self.client = actionlib.SimpleActionClient("/approach_table", ApproachTableAction)
        print 'Connecting to approach_table server'
        self.client.wait_for_server()
        print 'Connected to the approach_table server'

    def update(self):
        pass
    
    def reset(self):
        self.state = State.idle
        self.init()
