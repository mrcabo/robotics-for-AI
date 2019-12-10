import actionlib
import rospy
from my_msgs.msg import SimpleResult, SimpleAction

from common_action_server_functions import prepare_for_grasp, add_bounding_box_to_octomap, get_bounding_boxes
from moveit import MoveIt


class ActionServer(object):
    """
    roslaunch bounding_box_server bounding_box_server.launch robot:=tiago
    rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller <-- control joints
    """

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'grasp_action_server',
            SimpleAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.action_server.start()
        self.move_it = MoveIt('tiago')

    def attempt_grasp(self, bounding_box):
        # Close the fingers
        self.move_it.close_fingers()

        return self.move_it.grasp(
            x=bounding_box.x,
            y=bounding_box.y,
            z=bounding_box.z,
            rotation=bounding_box.yaw,
            z_max=bounding_box.z + bounding_box.height / 2,
            width=bounding_box.width
        )

    def grasp_box_flow(self, bounding_box):
        result = SimpleResult()

        print("Prepare for grasp: clear octomap, move head, move arm to side position")
        prepare_for_grasp(self.move_it)

        print("Add selected bounding box to octomap")
        add_bounding_box_to_octomap(self.move_it, bounding_box)

        print("Prepare for grasp: clear octomap, move head, move arm to side position")
        prepare_for_grasp(self.move_it)

        print("Attempt grasp")
        grasp_succes = self.attempt_grasp(bounding_box)

        if grasp_succes:
            print('Grasp succeeded')
            result.value = "SUCCES"
            self.action_server.set_succeeded(result)
        else:
            print('Grasp failed')
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
        result = self.grasp_box_flow(selected_bounding_box)
        return


if __name__ == '__main__':
    rospy.init_node('grasp_action_server')
    server = ActionServer()
    print "Grasp server started!"
    rospy.spin()