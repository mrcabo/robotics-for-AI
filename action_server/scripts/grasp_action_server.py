import time

import rospy
import actionlib
import random

from bounding_box_server.msg import BoundingBoxes
from my_msgs.msg import GraspResult, GraspAction
from moveit import MoveIt


class ActionServer(object):

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'grasp_action_server',
            GraspAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.action_server.start()

    @staticmethod
    def filter_bounding_boxes(bounding_boxes):
        min_volume = 0.00005
        max_volume = 0.005

        # print [(b.length, b.height, b.width,) for b in bounding_boxes]
        # print [b.length * b.height * b.width for b in bounding_boxes]

        bounding_boxes = filter(lambda b: min_volume < b.length * b.height * b.width < max_volume, bounding_boxes)

        return bounding_boxes

    def get_bounding_boxes(self, filter_out_bad_boxes=True):
        rospy.wait_for_service('enable_bounding_box_publisher')
        try:
            message = rospy.wait_for_message('/bounding_boxes', BoundingBoxes, 2.0)
        finally:
            rospy.wait_for_service('disable_bounding_box_publisher')

        return self.filter_bounding_boxes(message.bounding_boxes) if filter_out_bad_boxes else message.bounding_boxes

    def callback(self, goal):
        bounding_boxes = self.get_bounding_boxes()

        if len(bounding_boxes) == 0:
            self.action_server.set_aborted()
            return
        else:
            selected_bounding_box = bounding_boxes[0]


        move_it = MoveIt('tiago')

        # Add a precise collision box to the octomap
        move_it.add_collision_object(
            x=selected_bounding_box.x,
            y=selected_bounding_box.y,
            z=selected_bounding_box.z,
            rotation=selected_bounding_box.yaw,
            box_size=[selected_bounding_box.length, selected_bounding_box.width, selected_bounding_box.height]
        )

        # Close the fingers
        move_it.close_fingers()

        move_it.grasp(
            x=selected_bounding_box.x,
            y=selected_bounding_box.y,
            z=selected_bounding_box.z,
            rotation=selected_bounding_box.yaw,
            z_max=selected_bounding_box.z + selected_bounding_box.height / 2,
            width=selected_bounding_box.width
        )


        result = GraspResult()
        result.value = str(len(bounding_boxes))
        self.action_server.set_succeeded(result)

        ## mapping with rois blabla
        #
        #
        #
        # time.sleep(5)
        # result = AddIntegersResult()
        # result.value = goal.lhs + goal.rhs
        # r = random.random()
        # print r
        # if r <= 0.8:
        #     self.action_server.set_succeeded(result)
        # else:
        #     self.action_server.set_aborted(result)


if __name__ == '__main__':
    rospy.init_node('grasp_action_server')
    server = ActionServer()
    print "Grasp server started!"
    rospy.spin()
