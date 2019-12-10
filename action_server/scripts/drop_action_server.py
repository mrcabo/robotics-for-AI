import actionlib
import rospy
from my_msgs.msg import SimpleResult, SimpleAction

from common_action_server_functions import get_bounding_boxes, prepare_for_grasp
from moveit import MoveIt


class ActionServer(object):

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'drop_action_server',
            SimpleAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.action_server.start()
        self.move_it = MoveIt('tiago')

    def attempt_drop(self, bounding_box):
        print("Prepare for drop: clear octomap, move head, move arm to side position")
        prepare_for_grasp(self.move_it)

        success = self.move_it.move_to(
            x=bounding_box.x,
            y=bounding_box.y,
            z=bounding_box.z + (bounding_box.height / 2) + 0.2,  # TODO: This is 20cm from the center, it should be from the top
            rotation=bounding_box.yaw
        )

        result = SimpleResult()
        if success:
            # Drop the item
            self.move_it.open_fingers()
            self.move_it.close_fingers()
            self.move_it.delete_collision_object()

            result.value = "SUCCESS"
            self.action_server.set_succeeded(result)
        else:
            result.value = "FAILED"
            self.action_server.set_aborted(result)

        return result

    def callback(self, goal):
        bounding_boxes = get_bounding_boxes()

        if len(bounding_boxes) == 0:
            result = SimpleResult()
            result.value = "FAILED"
            self.action_server.set_aborted(result)
            return

        selected_bounding_box = bounding_boxes[0]

        print("Attempt drop")
        result = self.attempt_drop(selected_bounding_box)


if __name__ == '__main__':
    rospy.init_node('drop_action_server')
    server = ActionServer()
    print "Drop server started!"
    rospy.spin()
