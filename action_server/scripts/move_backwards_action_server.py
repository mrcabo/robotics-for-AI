import actionlib
import rospy
from base_control_server.srv import MoveBaseRequest, MoveBase
from my_msgs.msg import SimpleResult, SimpleAction

from common_action_server_functions import call_service
from moveit import MoveIt


class ActionServer(object):

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'move_backwards_server',
            SimpleAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.move_it = MoveIt('tiago')
        self.action_server.start()

    def callback(self, goal):
        move_base_req_msg = MoveBaseRequest()
        move_base_req_msg.meters = -.4

        print("Move backwards")
        result = SimpleResult()
        if call_service('move_base', MoveBase, move_base_req_msg):
            print("Moving arm to nav position")
            self.move_it.tiago_move_to_nav_position()
            result.value = "SUCCES"
            self.action_server.set_succeeded(result)
        else:
            result.value = "FAILED"
            self.action_server.set_aborted(result)


if __name__ == '__main__':
    rospy.init_node('move_backwards_server')
    server = ActionServer()
    print "Move backwards server started!"
    rospy.spin()
