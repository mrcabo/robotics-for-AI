import actionlib
import rospy
from my_msgs.msg import SimpleAction, SimpleGoal
from std_msgs.msg import String

from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class GraspSub(AbstractBehaviour):

    def init(self):
        self.client = actionlib.SimpleActionClient('grasp_action_server', SimpleAction)
        print 'Connecting to grasp_action_server'
        self.client.wait_for_server()
        print 'Connected to grasp_action_server'
        self.recognised_object = None
        self.text_to_speech_pub = rospy.Publisher('/speech', String, queue_size=3)

    def update(self):
        # When the state is start, send two integers to the action server
        if self.state == State.start:
            self.text_to_speech_pub.publish("Grasping the {}".format(self.recognised_object.class_name))
            goal = SimpleGoal()
            goal.s = str(self.recognised_object)
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
                self.fail(result.value)

    def reset(self):
        self.state = State.idle
        self.init()

    def start_pickup(self, item):
        self.recognised_object = item
        self.state = State.start

    def start(self):
        raise NotImplementedError("Use start pickup!")