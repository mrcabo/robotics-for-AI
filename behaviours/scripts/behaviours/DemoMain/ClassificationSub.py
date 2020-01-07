import actionlib
import rospy
from my_msgs.msg import ClassificationActionAction, ClassificationActionGoal
from std_msgs.msg import String

from behaviours.DemoMain.RecognisedObject import RecognisedObject
from utils.abstractBehaviour import AbstractBehaviour
from utils.state import State


class ClassificationSub(AbstractBehaviour):
    
    def init(self):
        self.client = actionlib.SimpleActionClient('classification_server', ClassificationActionAction)
        print 'Connecting to classification_server'
        self.client.wait_for_server()
        print 'Connected to the classification_server'
        self.result = None
        self.text_to_speech_pub = rospy.Publisher('/chatter', String, queue_size=3)

    def update(self):
        # When the state is start, send start signal to the action server
        if self.state == State.start:
            goal = ClassificationActionGoal()
            goal.start_signal = "START!"
            print "Sending start signal to action server"
            self.client.send_goal(goal)
            self.state = State.waiting
            self.result = None
        # When the state is waiting, ping the action server to see if it succeeded already
        elif self.state == State.waiting:
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                result = self.client.get_result()
                self.result = [RecognisedObject(s) for s in result.object_classifications]
                sorted_objects = [o.class_name for o in sorted(self.result, key=lambda o: o.center_x)]
                self.text_to_speech_pub.publish("I found {}".format(" ".join(sorted_objects)))

                print 'Action server succeeded with result: ', result.object_classifications
                self.finish()
            elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
                result = self.client.get_result()
                print 'Action server failed with result: ', result.object_classifications
                self.fail('Action server failed')

    def reset(self):
        self.state = State.idle
        self.init()
