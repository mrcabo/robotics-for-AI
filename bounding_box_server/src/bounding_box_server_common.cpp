#include "bounding_box_server/bounding_box_server.h"

//! Function to receive the latest Point Cloud Message.
/**
 * Uses waitForMessage() to receive a sensor_msgs::PointCloud2 message on the #point_cloud_topic_ topic, will wait 
 * for 1.0 second before returning a nullptr. 
 * \param point_cloud_message reference to which the newest Point Cloud will be returned to if one can be gathered.
 * \return true when a sensor_msgs was received, false if no sensor_msgs was received after 1.0 second.
 */
bool BoundingBoxServer::getPointCloudMessage(sensor_msgs::PointCloud2 &point_cloud_message) {
  boost::shared_ptr<const sensor_msgs::PointCloud2> point_cloud_message_shared_ptr;
  point_cloud_message_shared_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic_, ros::Duration(1.0));

  if (point_cloud_message_shared_ptr == nullptr) {
    ROS_WARN_STREAM("Could not receive PointCloud2 message after 1 second on topic: " << point_cloud_topic_);
    return false;
  }

  point_cloud_message = *point_cloud_message_shared_ptr;
  return true;
}

//! Function to transform a sensor_msgs::PointCloud2 to a different frame of reference.
/**
 * Will wait for transform to become available for 1 seconds, this will mostly fail if the server is just started and a request to transform
 * is giving right away, the transform_listener needs some time to initialize. 
 * It will convert all the points in the Point Cloud to the new frame of reference coordinate system, it will not change the size or dimensions of the 
 * Point Cloud.
 * \param point_cloud_message is the input Point Cloud Message.
 * \param transformed_point_cloud_message is the transformed Point Cloud Message.
 * \param transform_to_link indicates to which link (frame of reference) the Point Cloud Message needs to transform to.
 * \return true if the transform was available and possible, false if either transform was not available or could not transform.
 */
bool BoundingBoxServer::transformPointCloudMessageToLink(sensor_msgs::PointCloud2 &point_cloud_message,
                                                         sensor_msgs::PointCloud2 &transformed_point_cloud_message,
                                                         std::string transform_to_link) {
  try {
    transform_listener_.waitForTransform(transform_to_link, point_cloud_message.header.frame_id, ros::Time::now(), ros::Duration(1.0));
  } catch (...) {
    ROS_WARN_STREAM("No transform from " << transform_to_link << " to " << point_cloud_message.header.frame_id << " available");
    return false;
  }

  if (!transform_listener_.canTransform(transform_to_link, point_cloud_message.header.frame_id, point_cloud_message.header.stamp)) {
    ROS_WARN_STREAM("Can not transform Point Cloud from "  << point_cloud_message.header.frame_id << " to " << transform_to_link);
    return false;
  }

  pcl_ros::transformPointCloud(transform_to_link, point_cloud_message, transformed_point_cloud_message, transform_listener_);
  return true;
} 

//! Function to project the Point Cloud on the planar surface.
/** 
 * Will project all the points to have a zero z value. 
 * \param point_cloud Pointer to the input Point Cloud.
 * \param projected_point_cloud Pointer to the  Point Cloud that should contain the projected point_cloud on the planar surface.
 */
void BoundingBoxServer::projectPointCloudOnPlane(PointCloudPtr point_cloud, PointCloudPtr projected_point_cloud) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0.0f;
  coefficients->values[2] = 1.0f; // Z-axis 

  /*
  project_inliers_ is a pcl::ProjectInliers<Point> class object,
  http://docs.pointclouds.org/1.9.1/classpcl_1_1_project_inliers.html

  Which inherits from PCLBase<Point>,
  http://docs.pointclouds.org/1.9.1/classpcl_1_1_p_c_l_base.html

  - Set the input cloud, point_cloud
  - Set the model type, pcl::SACMODEL_PLANE
  - Set the coefficients, coefficients
  - Filter the point cloud, use *project_point_cloud
  */
}

//! Get the eigen vectors of a Point Cloud.
/**
 * Assumes the Point Cloud has been projected to the planar surface in order to work correctly.
 * \param point_cloud Pointer to a Point Cloud that has been project on a planar surface.
 * \return A matrix containing the 3 eigen vectors in each column.
 */ 
Eigen::Matrix3f BoundingBoxServer::getEigenVectors(PointCloudPtr point_cloud) {
  /*
  pca_ is a pcl::PCA<Point> class object,
  http://docs.pointclouds.org/1.9.1/classpcl_1_1_p_c_a.html

  Which inherits from PCLBase<Point>,
  http://docs.pointclouds.org/1.9.1/classpcl_1_1_p_c_l_base.html

  - Set the input cloud, point_cloud
  - return the eigenvectors 
  */
}

//! Get the angle between the eigen vector and a (1,0) vector.
/** 
 * Calculate the angle between two vectors, base_link_vector is a vector (1,0) representing the direction of the robot base.
 * \param eigen_vector the eigen vector from the object representing the yaw rotation.
 * \return the angle between the base_link_vector (1,0) and the eigen_vector.
 */
float BoundingBoxServer::getAngle(Eigen::Vector3f eigen_vector) {
  Eigen::Vector2f object_vector = eigen_vector.head<2>(); // Only use x and y
  Eigen::Vector2f base_link_vector;
  base_link_vector << 1.0f, 0.0f; // x = 1.0, y = 0.0, pointing forwards

  /* 
  return the angle between the object_vector and base_link_vector
  */
}

//! Get the center point of the given Point Cloud.
/**
 * Calculate and return the center point (x, y, z) of the given Point Cloud.
 * \param point_cloud Pointer to the input Point Cloud.
 * \return vector containing the center x, y, z coordinates.
 */
Eigen::Vector3f BoundingBoxServer::getCenterPointCloud(PointCloudPtr point_cloud) {
  /*
  There exists a typedef for a pcl::PointT, named Point
  There exists a function pcl::getMinMax3D to get the min and max values of the point cloud,
  http://docs.pointclouds.org/1.7.0/group__common.html#ga3166f09aafd659f69dc75e63f5e10f81

  Create an Eigen::Vector3f centroid_vector; 
  Assign the center of the point cloud to the centroid_vector.
  either by centroid_vector << x, y, z; 
  or centroid_vector.x() = x;
  centroid_vector.y() = y;
  centroid.vector.z() = z;

  return the centroid_vector
  */
}

//! Transforms a given Point Cloud to the origin, including rotation.
/**
 * Rotates and translates a given Point Cloud based on the centroid_vector (translation), and the yaw (rotation) to the origin (0, 0, 0).
 * \param point_cloud Pointer to the input Point Cloud.
 * \param centered_point_cloud Pointer to the transformed Point Cloud to the origin (0, 0, 0) and -yaw rotation.
 * \param centroid_vector the vector pointing to the center of point_cloud.
 * \param angle the yaw rotation of point_cloud in radians.
 */
void BoundingBoxServer::transformPointCloudToCenter(PointCloudPtr point_cloud, PointCloudPtr centered_point_cloud, Eigen::Vector3f centroid_vector, float angle) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  angle = -angle; // We want to rotate the Point Cloud back to the origin

  // Rotation matrix for the z-axis
  transform(0,0) = std::cos(angle);
  transform(0,1) = -std::sin(angle);
  transform(1,0) = std::sin(angle);
  transform(1,1) = std::cos(angle); 
  // Translation back to origin (0, 0, 0)
  transform.block<3,1>(0,3) = -1.0f * (transform.block<3,3>(0,0) * centroid_vector.head<3>());
  pcl::transformPointCloud(*point_cloud, *centered_point_cloud, transform);
}

//! Gets the dimensions length, width, height of the given Point Cloud.
/**
 * Calculates the min and max points, from which it extract the center of each axis.
 * \param point_cloud Pointer to the input Point Cloud of which to calculate the dimensions, the Point Cloud should be transformed to the origin.
 * \param bounding_box the return value to include the length, width, and height of point_cloud.
 */
void BoundingBoxServer::getDimensions(PointCloudPtr point_cloud, bounding_box_server::BoundingBox &bounding_box) {
  /*
  There exists a typedef for a pcl::PointT, named Point
  There exists a function pcl::getMinMax3D to get the min and max values of the point cloud,
  http://docs.pointclouds.org/1.7.0/group__common.html#ga3166f09aafd659f69dc75e63f5e10f81

  assign the length, x axis, to bounding_box.length
  assign the width, y axis, to bounding_box.width
  assign the height, z axis, to bounding_box.height
  */
}
