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
import cv2


class ActionServer(object):
    image_topic = "xtion/rgb/image_raw"
    model_path = os.path.join(os.environ['HOME'], 'network_model', '16_8_128_7_convnet.h5')
    image_save_path = os.path.join(os.environ['HOME'], 'saved_images')
    input_size = (64, 64)
    class_names = ["crazyflie", "erazer_box", "evergreen", "jetson", "powerbank", "raspicam", "whitebox"]

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'classification_server',
            ClassificationActionAction,
            execute_cb=self.callback,
            auto_start=False
        )
        self.model = tf.keras.models.load_model(self.model_path)
        self.action_server.start()

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

    @staticmethod
    def cut_and_scale(image, roi, size=64):
        # clip the rois to image width/height
        width = image.shape[1]
        height = image.shape[0]
        roi.top = min(max(0, roi.top), height)
        roi.bottom = min(max(0, roi.bottom), height)
        roi.left = min(max(0, roi.left), width)
        roi.right = min(max(0, roi.right), width)

        # cut
        image_crop = image[roi.top:roi.bottom, roi.left:roi.right]
        # scale
        scaled = cv2.resize(image_crop, (size, size))
        return scaled

    def get_rois(self):
        rois = self.requestROI()
        object_rois = sorted(copy(rois.object_rois), key=lambda r: r.left)
        print('Number of ROIs: {}'.format(len(object_rois)))
        return object_rois

    def get_image(self):
        bridge = CvBridge()
        image_msg = rospy.wait_for_message(self.image_topic, Image)
        image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image = image.astype(np.float32) / 255.
        return image

    def save_images(self, scaled_images, predictions, predicted_class_names):
        for idx, image, prediction, class_name in zip(range(scaled_images.shape[0]), scaled_images, predictions, predicted_class_names):
            filename = "{}_{}_{:.2f}.jpg".format(idx, class_name, max(prediction) * 100)
            cv2.imwrite(os.path.join(self.image_save_path, filename), (image * 255).astype('int'))

    def predict_one_roi(self, image, roi):
        # Define the shift off roi
        roi_width = abs(roi.right - roi.left)
        roi_height = abs(roi.top - roi.bottom)
        print("roi width/height %d/%d" % (roi_width, roi_height))
        x_diffs = [-(roi_width // 5), 0, (roi_width // 5)]
        y_diffs = [-(roi_height // 5), 0, (roi_height // 5)]

        # Cutout this roi and the area around it 9 times
        cropped_and_scaled_images = []
        for d_x in x_diffs:
            for d_y in y_diffs:
                temp_roi = copy(roi)
                temp_roi.left += d_x
                temp_roi.right += d_x
                temp_roi.top += d_y
                temp_roi.bottom += d_y
                cropped_and_scaled_images.append(self.cut_and_scale(image, temp_roi))
        cropped_and_scaled_images = np.array(cropped_and_scaled_images)
        # self.save_images(cropped_and_scaled_images, [[roi.left, 0]] * 9, [roi.right] * 9)

        # Predict for all 9 cutouts
        prediction = self.model.predict(cropped_and_scaled_images)

        hist = np.zeros(len(self.class_names))
        for p in prediction:
            hist[np.argmax(p)] += 1
        hist /= 9.
        return hist

    def callback(self, goal):
        # Retrieve image from image topic
        image = self.get_image()
        # Get regions of interest using the pointclouds
        rois = self.get_rois()
        # Average over 9 prediction made on an area around th roi
        predictions = [self.predict_one_roi(image, roi) for roi in rois]
        # predictions -> class names
        predicted_class_names = [self.class_names[np.argmax(prediction)] for prediction in predictions]
        # Save the final classification
        self.save_images(np.array([self.cut_and_scale(image, roi) for roi in rois]), predictions, predicted_class_names)

        # Build the result message
        result = ClassificationActionResult()
        result.object_classifications = predicted_class_names
        self.action_server.set_succeeded(result)


if __name__ == '__main__':
    """
    Setup these folder on home network_model/ and saved_images/
    """
    rospy.init_node('classification_server')
    server = ActionServer()
    rospy.spin()
