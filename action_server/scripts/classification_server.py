import os
from copy import copy

import actionlib
import numpy as np
import rospy
import tensorflow as tf
from cv_bridge import CvBridge
from my_msgs.msg import ClassificationActionAction, ClassificationActionResult
from point_cloud_functions.srv import GetObjectROI, GetObjectROIRequest
from sensor_msgs.msg import Image


class ActionServer(object):
    image_topic = "xtion/rgb/image_raw"
    path = os.path.join(os.environ['HOME'], 'network_model', '16_8_128_7_convnet.h5')
    class_names = ["crazyflie", "erazer_box", "evergreen", "jetson", "powerbank", "raspicam", "whitebox"]

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'classification_server',
            ClassificationActionAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.action_server.start()
        self.model = tf.keras.models.load_model(self.path)

    @staticmethod
    def requestROI():
        ## makes an roi request....
        ## much lines, many parameters. Keep parameters as is.
        cl = rospy.ServiceProxy('get_object_roi', GetObjectROI)
        request = GetObjectROIRequest()
        request.transform_to_link = "base_link"
        request.cluster_distance = 0.05
        request.surface_threshold = 0.015
        request.filter_x = True
        request.min_x = 0.1
        request.max_x = 0.8
        request.filter_y = True
        request.min_y = -0.3
        request.max_y = 0.3
        request.filter_z = True
        request.min_z = 0.3
        request.max_z = 2.0
        request.min_cluster_points = 10
        request.max_cluster_points = 100000
        request.voxelize_cloud = True
        request.leaf_size = 0.01
        return cl(request)  ## makes the request and returns the response

    def get_cropped_images(self):
        rois = self.requestROI()
        bridge = CvBridge()
        image_msg = rospy.wait_for_message(self.image_topic, Image)
        image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image = image.astype(np.float32) / 255.

        return np.array([copy(image[roi.top:roi.bottom, roi.left:roi.right]) for roi in rois.object_rois])
        # for roi in rois.object_rois:
        #     left = roi.left  ## x coordinate
        #     right = roi.right  ## x coordinate
        #     top = roi.top  ## y coordinate
        #     bottom = roi.bottom  ## y coordinate
        #     # print '(', left, ', ', top, ')', '(', right, ', ', bottom, ')'
        #     cropped_images.append(copy(image[top:bottom, left:right]))
        # return np.array(cropped_images)

    def callback(self, goal):
        cropped_images = self.get_cropped_images()
        print(cropped_images.shape)
        predictions = self.model.predict(cropped_images)
        print(predictions.shape)
        result = ClassificationActionResult()
        result.object_classifications = [self.class_names[np.argmax(prediction)] for prediction in predictions]
        self.action_server.set_succeeded(result)
        # self.action_server.set_aborted(result)


if __name__ == '__main__':
    rospy.init_node('classification_server')
    server = ActionServer()
    rospy.spin()
