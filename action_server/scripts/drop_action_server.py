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

        print("Moving arm in position for drop")

        success = self.move_it.move_to_drop(
            x=bounding_box.x,
            y=bounding_box.y,
            z=bounding_box.z + (bounding_box.height / 2)
        )

        result = SimpleResult()
        if success:
            # Drop the item
            self.move_it.open_fingers()
            rospy.sleep(2)
            self.move_it.close_fingers()
            for _ in range(5):  # Just to be save remove it 5 times bacause of bad cpus:(
                self.move_it.delete_collision_object()
                rospy.sleep(0.1)

            print('Drop succeeded')
            result.value = "SUCCESS"
            self.action_server.set_succeeded(result)
        else:
            print('Drop failed')
            result.value = "MOVE ARM FAILED"
            self.action_server.set_aborted(result)

        return result

    def callback(self, goal):
        bounding_boxes = get_bounding_boxes(get_big_box=True)

        if len(bounding_boxes) == 0:
            result = SimpleResult()
            result.value = "NO BOUNDING BOXES"
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
