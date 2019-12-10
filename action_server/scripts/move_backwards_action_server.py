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

    def move_arm_to_nav_with_recovery(self):
        print("Moving arm to nav position")
        # Try to put arm in nav position
        if self.move_it.tiago_move_to_nav_position():
            return True
        else:
            print("Move to nav position failed, trying recovery behaviours")
            self.move_it.close_fingers()
            arm_to_side_succes = self.move_it.tiago_move_to_side_position()
            return self.move_it.tiago_move_to_nav_position() if arm_to_side_succes else False

    def callback(self, goal):

        print("Move backwards")
        result = SimpleResult()

        # Try to move back
        move_base_req_msg = MoveBaseRequest()
        move_base_req_msg.meters = -.4
        if call_service('move_base', MoveBase, move_base_req_msg):
            call_service('clear_octomap')
            if self.move_arm_to_nav_with_recovery():
                result.value = "SUCCES"
                self.action_server.set_succeeded(result)
            else:
                result.value = "ARM FAILED"
                self.action_server.set_aborted(result)
        else:
            result.value = "MOVE FAILED"
            self.action_server.set_aborted(result)


if __name__ == '__main__':
    rospy.init_node('move_backwards_server')
    server = ActionServer()
    print "Move backwards server started!"
    rospy.spin()
