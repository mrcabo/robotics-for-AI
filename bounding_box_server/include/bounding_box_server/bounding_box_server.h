#ifndef _H_BOUNDING_BOX_SERVER__
#define _H_BOUNDING_BOX_SERVER__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl-1.9/pcl/point_cloud.h>
#include <pcl-1.9/pcl/point_types.h>
#include <pcl-1.9/pcl/filters/passthrough.h>
#include <pcl-1.9/pcl/filters/project_inliers.h>
#include <pcl-1.9/pcl/filters/voxel_grid.h>
#include <pcl-1.9/pcl/filters/extract_indices.h>
#include <pcl-1.9/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.9/pcl/segmentation/extract_clusters.h>
#include <pcl-1.9/pcl/search/kdtree.h>
#include <pcl-1.9/pcl/common/pca.h>
#include <pcl-1.9/pcl/common/transforms.h>
#include <pcl-1.9/pcl/common/centroid.h>

#include <Eigen/Eigen>
#include <vector>

#include <bounding_box_server/BoundingBox.h>
#include <bounding_box_server/BoundingBoxes.h>
#include <std_srvs/Empty.h>

#include "bounding_box_publisher.h"

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr; 

class BoundingBoxServer {

  //! NodeHandle for creating publishers and services
  ros::NodeHandle nh_;

  //! Point cloud topic from which to receive the Point Cloud Sensor data
  std::string point_cloud_topic_;
  //! Transform listener used for transforming the Point Cloud data to a different coordinate system
  tf::TransformListener transform_listener_;
  //! Publisher to publish the bounding boxes with the BoundingBoxes message
  BoundingBoxPublisher bounding_box_publisher_;
  //! Service to enable the publishing of the bounding boxes
  ros::ServiceServer enable_bounding_box_publisher_service_;
  //! Service to disable the publishing of the bounding boxes
  ros::ServiceServer disable_bounding_box_publisher_service_;

  //! PCL Class for filtering out parts of the Point Cloud
  pcl::PassThrough<Point> pass_through_filter_;
  //! PCL Class for segmenting the Point Cloud, used for remove the table surface
  pcl::SACSegmentation<Point> sac_segmentation_;
  //! PCL Class for extracting specific points based on indices 
  pcl::ExtractIndices<Point> extract_indices_;
  //! PCL Class for extracting clusters from a Point Cloud
  pcl::EuclideanClusterExtraction<Point> cluster_extraction_;
  //! PCL Class for project a Point Cloud
  pcl::ProjectInliers<Point> project_inliers_;
  //! PCL Class for reducing the Point Cloud in based on a voxel filter
  pcl::VoxelGrid<Point> voxel_grid_; 
  //! PCL Class for performing Principal Component Analasys
  pcl::PCA<Point> pca_;
  //! Timer thats run the algorithms leading to publishing the bounding boxes data
  ros::Timer update_timer_;
  //! String that indices the link to transform the Point Cloud data to
  const std::string transform_to_link_;

  public: 
    BoundingBoxServer(ros::NodeHandle &nh);

  private:
    void updateTimerEvent(const ros::TimerEvent &);
    bool enableBoundingBoxPublisher(std_srvs::EmptyRequest &, std_srvs::EmptyResponse &);
    bool disableBoundingBoxPublisher(std_srvs::EmptyRequest &, std_srvs::EmptyResponse &);
    bool getPointCloudMessage(sensor_msgs::PointCloud2 &point_cloud_message);
    bool transformPointCloudMessageToLink(sensor_msgs::PointCloud2 &point_cloud_message, sensor_msgs::PointCloud2 &transformed_point_cloud_message, std::string transform_to_link);
    void passThroughFilter(PointCloudPtr point_cloud, PointCloudPtr filtered_cloud, std::string field_name, float min_value, float max_value);
    void removeTable(PointCloudPtr point_cloud, PointCloudPtr tableless_point_cloud, float distance_threshold);
    void voxelizePointCloud(PointCloudPtr point_cloud, PointCloudPtr voxelized_point_cloud, float leaf_size = 0.02f);
    void extractClusters(PointCloudPtr point_cloud, std::vector<PointCloudPtr> &clusters, float cluster_tolerance);
    void projectPointCloudOnPlane(PointCloudPtr point_cloud, PointCloudPtr projected_point_cloud);
    Eigen::Matrix3f getEigenVectors(PointCloudPtr point_cloud); 
    float getAngle(Eigen::Vector3f eigen_vector);
    Eigen::Vector3f getCenterPointCloud(PointCloudPtr point_cloud);
    void transformPointCloudToCenter(PointCloudPtr point_cloud, PointCloudPtr center_cloud, Eigen::Vector3f centroid_vector, float angle);
    void getDimensions(PointCloudPtr point_cloud, bounding_box_server::BoundingBox &bounding_box);
};

#endif