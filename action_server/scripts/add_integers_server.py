import time

import rospy
import actionlib
import random
from my_msgs.msg import AddIntegersAction, AddIntegersResult


class ActionServer(object):

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'add_integers_server',
            AddIntegersAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.action_server.start()

    def callback(self, goal):
        time.sleep(5)
        result = AddIntegersResult()
        result.value = goal.lhs + goal.rhs
        if random.random() <= 0.8:
            self.action_server.set_succeeded(result)
        else:
            self.action_server.set_aborted(result)


if __name__ == '__main__':
    rospy.init_node('add_integers_server')
    server = ActionServer()
    rospy.spin()
