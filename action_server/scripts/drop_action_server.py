import time

import rospy
import actionlib
import random

from base_control_server.srv import MoveBaseRequest, MoveBase
from bounding_box_server.msg import BoundingBoxes
from head_controller.srv import MoveHead
from my_msgs.msg import GraspResult, GraspAction
from std_srvs.srv import Empty
from moveit import MoveIt

class ActionServer(object):

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'drop_action_server',
            GraspAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.action_server.start()
        self.move_it = MoveIt('tiago')

    def switch_bounding_box_publisher(self, on=True):
        service_name = 'enable_bounding_box_publisher' if on else 'disable_bounding_box_publisher'
        rospy.wait_for_service(service_name)
        try:
            handler = rospy.ServiceProxy(service_name, Empty)
            handler()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    @staticmethod
    def filter_bounding_boxes(bounding_boxes):
        # TODO: This one probably needs to be bigger in real robot. Maybe mix it with object recognizer
        min_volume = 0.00005
        max_volume = 0.005
        min_z = 0.5

        # print [(b.length, b.height, b.width,) for b in bounding_boxes]
        # print [b.length * b.height * b.width for b in bounding_boxes]

        bounding_boxes = filter(lambda b: min_volume < b.length * b.height * b.width < max_volume, bounding_boxes)
        bounding_boxes = filter(lambda b: b.z > min_z, bounding_boxes)

        return bounding_boxes

    def get_bounding_boxes(self, filter_out_bad_boxes=True):
        self.switch_bounding_box_publisher(True)
        try:
            message = rospy.wait_for_message('/bounding_boxes', BoundingBoxes, 3.0)
        finally:
            self.switch_bounding_box_publisher(False)

        return self.filter_bounding_boxes(message.bounding_boxes) if filter_out_bad_boxes else message.bounding_boxes

    def move_head(self, pitch, yaw):
        rospy.wait_for_service('move_head')
        try:
            move_head_handler = rospy.ServiceProxy('move_head', MoveHead)
            move_head_handler(pitch, yaw)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def prepare_for_grasp(self, move_it):
        rospy.wait_for_service('clear_octomap')  # TODO: will this really call the clear_octomap??
        self.move_head(.6, 0)
        self.move_head(.6, -1)
        self.move_head(.6, 1)
        self.move_head(.6, 0)
        move_it.tiago_move_to_side_position()

    def attempt_drop(self, move_it, bounding_box):
        # Close the fingers
        self.move_it.close_fingers()

        return self.move_it.move_to(
            x=bounding_box.x,
            y=bounding_box.y,
            z=bounding_box.z + 0.2,  # TODO: This is 20cm from the center, it should be from the top
            rotation=bounding_box.yaw
        )

    def callback(self, goal):
        result = GraspResult()
        bounding_boxes = self.get_bounding_boxes()

        if len(bounding_boxes) == 0:
            result.value = "FAILED"
            self.action_server.set_aborted(result)
        else:
            selected_bounding_box = bounding_boxes[0]

            print("Prepare for drop: clear octomap, move head, move arm to side position")
            self.prepare_for_grasp(self.move_it)

            print("Attempt drop")
            grasp_succes = self.attempt_drop(self.move_it, selected_bounding_box)

            if grasp_succes:
                result.value = str(len(bounding_boxes))
                self.action_server.set_succeeded(result)
            else:
                self.action_server.set_aborted(result)


if __name__ == '__main__':
    rospy.init_node('add_integers_server')
    server = ActionServer()
    print "Drop server started!"
    rospy.spin()