import actionlib
import rospy
from my_msgs.msg import SimpleResult, SimpleAction


import sys
sys.path.append("../../behaviours")


from scripts.behaviours.DemoMain.RecognisedObject import RecognisedObject
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

        z_top = bounding_box.z + bounding_box.height / 2

        return self.move_it.grasp(
            x=bounding_box.x + 0.0075,
            y=bounding_box.y,
            z=z_top - 0.07,
            rotation=bounding_box.yaw,
            z_max=z_top - 0.05,
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
            result.value = "MOVE ARM FAILED"
            self.action_server.set_aborted(result)
        return result

    def roi_bb_dis(self, roi, bb):
        dis = (bb.x-roi.center_x) ** 2 + (bb.y-roi.center_y) ** 2 + (bb.z-roi.center_z) ** 2
        print("distance = {:.4f}".format(dis))
        return dis

    def get_closest_bounding_box(self, goal_object, bounding_boxes, max_distance=0.03):
        sorted_box_distances = sorted([(box, self.roi_bb_dis(goal_object, box)) for box in bounding_boxes],
                                      key=lambda p: p[1])
        filtered_box_distances = filter(lambda p: p[1] < max_distance, sorted_box_distances)
        return filtered_box_distances[0][0] if filtered_box_distances else None

    def callback(self, goal):
        goal_object = RecognisedObject(goal.s)
        bounding_boxes = get_bounding_boxes(get_big_box=False)

        if len(bounding_boxes) == 0:
            result = SimpleResult()
            result.value = "NO BOUNDING BOXES"
            self.action_server.set_aborted(result)
            return

        selected_bounding_box = self.get_closest_bounding_box(goal_object, bounding_boxes)
        if selected_bounding_box is None:
            result = SimpleResult()
            result.value = "no_matching_bounding_box"
            self.action_server.set_aborted(result)
            return

        result = self.grasp_box_flow(selected_bounding_box)
        return


if __name__ == '__main__':
    rospy.init_node('grasp_action_server')
    server = ActionServer()
    print "Grasp server started!"
    rospy.spin()
