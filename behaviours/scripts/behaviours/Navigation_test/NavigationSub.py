import actionlib

from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy


class NavigationSub(AbstractBehaviour):
    
    def init(self):
        self.pos = None
        self.orientation = None
        if not hasattr(self, 'client'):
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print 'Connecting to server'
        self.client.wait_for_server()
        print 'Connected to the server'

    def update(self):
        if self.state == State.start:
            self.client.send_goal(self.build_goal_message())
            self.set_state(State.navigating)

        elif self.state == State.navigating:
            client_state = self.client.get_state()
            if client_state == actionlib.GoalStatus.SUCCEEDED:
                result = self.client.get_result()
                print('Navigation completed, with ressult %s' % result)
                self.finish()
            elif client_state == actionlib.GoalStatus.ABORTED:
                result = self.client.get_result()
                print('Navigation failed with result %s' % result)
                self.fail(reason="Action server returned ABORTED status with result %s" % result)
    
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