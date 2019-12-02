#include "bounding_box_server/bounding_box_server.h"

//! Callback for timer event.
/**
 * This function will gather the Point Cloud Sensor message, transforms it to the correct link,
 * converts it to PCL data-structure, performs voxelization, passthrough and segmentation filters on the Point Cloud.
 * Next, clustering will happen, and for each cluster the orientation and transformation are calculated. The bounding box dimensions, position and orientation 
 * are then published, and a marker array for visualization is published.
 */
void BoundingBoxServer::updateTimerEvent(const ros::TimerEvent &) {
  sensor_msgs::PointCloud2 point_cloud_message;

  // Retreive a Point Cloud Message from the point_cloud_topic_ topic 
  if (!getPointCloudMessage(point_cloud_message)) {
    return;
  }

  // Transform the Point Cloud Message to a different frame of reference
  if (!transformPointCloudMessageToLink(point_cloud_message, point_cloud_message, transform_to_link_)) {
    return;
  }

  // Convert Point Cloud Message to a PCL Point Cloud
  PointCloudPtr point_cloud(new PointCloud());
  pcl::fromROSMsg(point_cloud_message, *point_cloud);

  voxelizePointCloud(point_cloud, point_cloud); // Reduce the Point Cloud size
  passThroughFilter(point_cloud, point_cloud, "z", 0, 1.2);  // // Mostly to remove the floor from the Point Cloud
  removeTable(point_cloud, point_cloud, 0.01f); // Remove the planar (should be table) surface

  std::vector<PointCloudPtr> cluster_clouds;
  extractClusters(point_cloud, cluster_clouds, 0.05f); // Extract the clusters and keep them in a dynamic array (vector)

  if (cluster_clouds.size() == 0) { // Could be that no clusters are found
    ROS_WARN("No clusters found");
    return;
  }

  std::vector<bounding_box_server::BoundingBox> bounding_boxes;

  for (auto cluster_cloud: cluster_clouds) { // For each Point Cloud in the dynamic array (vector) do
    PointCloudPtr projected_point_cloud(new PointCloud());
    projectPointCloudOnPlane(cluster_cloud, projected_point_cloud); // Flatten the Point Cloud on the planar surface

    Eigen::Matrix3f eigen_vectors = getEigenVectors(projected_point_cloud); // Retreive the 3 eigen vectors, in a column based matrix
    float angle = getAngle(eigen_vectors.col(0));
    
    Eigen::Vector3f centroid_vector = getCenterPointCloud(cluster_cloud); // Calculate the center of the Point Cloud, in a Vector3f format
    
    PointCloudPtr centered_point_cloud(new PointCloud());
    transformPointCloudToCenter(cluster_cloud, centered_point_cloud, centroid_vector, angle); // Rotate and Translate the Point Cloud to the origin (0, 0, 0)

    bounding_box_server::BoundingBox bounding_box;
    
    bounding_box.x = centroid_vector.x();
    bounding_box.y = centroid_vector.y();
    bounding_box.z = centroid_vector.z();
    bounding_box.yaw = angle;

    getDimensions(centered_point_cloud, bounding_box); // Calculate the length, width, and height of the Transformed Point Cloud
    bounding_boxes.push_back(bounding_box);
  }

  // Publish the bounding boxes to make them available for other nodes, and publishes a visualization marker to visualize the bounding boxes in Rviz
  bounding_box_publisher_.publishBoundingBoxes(bounding_boxes);
}
