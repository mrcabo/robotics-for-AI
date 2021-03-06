#include "bounding_box_server/bounding_box_server.h"

//! Filters parts of the Point Cloud per axis based on min and max values.
/**
 * Uses the pcl::PassThrough filter to filter out parts of the Point Cloud. 
 * All points on the specified axis between min_value and max_value will be contained in the output Point Cloud, 
 * point below min_value and above max_value will therefore be removed.
 * \param point_cloud Pointer to input Point Cloud.
 * \param filtered_point_cloud Pointer to the output filtered Point Cloud.
 * \param field_name the axis on which to filter, either "x", "y", or "z".
 * \param min_value the min value off all the filtered points.
 * \param max_value the max value off all the filtered points.
 */
void BoundingBoxServer::passThroughFilter(PointCloudPtr point_cloud, PointCloudPtr filtered_point_cloud,
                                          std::string field_name, float min_value, float max_value) {
  pass_through_filter_.setInputCloud(point_cloud); // Pointer to the input Point Cloud
  pass_through_filter_.setFilterFieldName(field_name); // Indicates on which axis to filter
  pass_through_filter_.setFilterLimits(min_value, max_value); 
  pass_through_filter_.filter(*filtered_point_cloud);  // The output of the filter                               
}

//! Removes a flat surface from the Point Cloud.
/**
 * Removes 1 flat surface, which should be the table, from the Point Cloud. It uses RANSAC to find points belonging to a possible 
 * planar surface, and checks if the points lays within a theshold to determine if it belongs to the surface or not. 
 * \param point_cloud Pointer to the input Point Cloud, should only contain one large planar surface (the table), assumes there is no floor visible.
 * \param tableless_point_cloud Pointer to the Point Cloud without a table surface in it.
 * \param distance_threshold determines the threshold in meters when a points belongs to a possible planar surface or not.
 */
float BoundingBoxServer::removeTable(PointCloudPtr point_cloud, PointCloudPtr tableless_point_cloud, float distance_threshold) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());  // Needed for the segmentiation
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());                 // Needed for the segmentation

  /*
  sac_segmentation_ is a pcl::SACSegmentation<Point> class object,
  http://docs.pointclouds.org/trunk/classpcl_1_1_s_a_c_segmentation.html

  Which inherits the PCLBase class
  http://docs.pointclouds.org/trunk/classpcl_1_1_p_c_l_base.html

  - You need to set the input point cloud, point_cloud.
  - You need to set a distance threshold, distance_threshold, how close is a point when it still belongs to the surface
  - Set the method type to pcl::SAC_RANSAC
  - Set the model type to pcl::SACMODEL_PLANE
  - Segment using the inliers and coefficients objects (they are pointers so use a * before the name)
    Whilst both parameters are required, only inliers will be used later
    The inliers object should now contain all the indexes of the points that are part of the table
  */

  // Optional
  //sac_segmentation_.setOptimizeCoefficients (true);

  // Mandatory
  sac_segmentation_.setModelType (pcl::SACMODEL_PLANE);
  sac_segmentation_.setMethodType (pcl::SAC_RANSAC);
  sac_segmentation_.setDistanceThreshold (distance_threshold);

  sac_segmentation_.setInputCloud (point_cloud);
  sac_segmentation_.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return -1;
  }

  /*
  extract_indices_ is a pcl::ExtractIndices<Point> class object,
  http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html

  which inherits the PCLBase class,

  and the pcl::FilterIndices<Point> class,
  http://docs.pointclouds.org/trunk/classpcl_1_1_filter_indices.html

  - You need to set the input point cloud
  - You need to set the indicies
  - You need to set whether the to keep the indices, or remove the indices from the point cloud
  - Filter the point cloud, and write the new point cloud to *tableless_point_cloud
  */

  extract_indices_.setInputCloud (point_cloud);
  extract_indices_.setIndices (inliers);

  // new
  PointCloudPtr table_pc (new PointCloud());
  extract_indices_.setNegative (false);
  extract_indices_.filter (*table_pc);
  Point min, max;
  pcl::getMinMax3D(*table_pc, min, max);
  float highest_z_of_table = max.z;

  std::cerr << "PointCloud representing the planar component: " << table_pc->width * table_pc->height << " data points." << std::endl;

  // proven to work
  extract_indices_.setNegative (true);
  extract_indices_.filter (*tableless_point_cloud);
  return highest_z_of_table;
}

//! Given a Point Cloud, extract the clusters as seperate Point Clouds.
/**
 * Given a Point Cloud extract the clusters and put them in their own Point Cloud. 
 * \param point_cloud Pointer to the input Point Cloud. The Point Cloud should have the table surface removed.
 * \param clusters a vector of Pointers to Point Clouds, each Point Cloud represents a cluster.
 * \param cluster_tolerance the distance between two points that indices when they become their seperate cluster.
 */
void BoundingBoxServer::extractClusters(PointCloudPtr point_cloud, std::vector<PointCloudPtr> &clusters, float cluster_tolerance) {
  pcl::search::KdTree<Point>::Ptr search_tree(new pcl::search::KdTree<Point>()); // Used for quickly searching the Point Cloud structure
  search_tree->setInputCloud(point_cloud); // Pointer to the input Point Cloud 

  std::vector<pcl::PointIndices> cluster_indices; // vector for storing the indices for each cluster

  //cluster_extraction_ is a pcl::EuclideanClusterExtraction<Point> class object,

  //  - Set the input point cloud, point_cloud
  cluster_extraction_.setInputCloud (point_cloud);
  //  - Set the search method, search_tree
  cluster_extraction_.setSearchMethod (search_tree);
  //  - Set the cluster tolerance, cluster_tolerance
  cluster_extraction_.setClusterTolerance (cluster_tolerance); // 0.02 is 2 cm
  //  - Set the min and max cluster sizes, I recommend between 10 and 50000
  cluster_extraction_.setMinClusterSize (10);
  cluster_extraction_.setMaxClusterSize (50000);
  //  - Extract the clusters with the parameter, cluster_indices
  cluster_extraction_.extract (cluster_indices);
  //    cluster_indices is now a vector, where each items is another vector of all the indices that belong to a single cluster


  /*
  for each cluster in cluster_indices, do
    - create a new point cloud ptr, PointCloudPtr cluster_cloud(new PointCloud())

    for each index value of the PointIndices, do
      - add the point at position index, from the point_cloud, to the cluster_cloud.
      You can access the points via e.g. point_cloud->points, points is a vector, from which you can get an item via index, points[index],
      or add a new item at the back, points.push_back(point)

    Add the newly create cluster_cloud to the clusters vector (parameter for the function), use push_back
  */
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    PointCloudPtr cluster_cloud(new PointCloud());
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cluster_cloud->points.push_back (point_cloud->points[*pit]); //*
    // Maybe we need to set with, height and is_dense read: http://pointclouds.org/documentation/tutorials/cluster_extraction.php
    clusters.push_back(cluster_cloud);
  }
}

//! Voxelizes the Point Cloud.
/**
 * Reduces the Point Cloud size by creating voxels of multiple points based on the leaf size, each points within 
 * the voxel will be indicated just by that one voxel. This will mostly help to reduce the calculation time of other 
 * algorithms.
 * \param point_cloud Pointer to the Point Cloud.
 * \param voxelized_point_cloud Pointer to the voxelized Point Cloud based on leaf_size parameter.
 * \param leaf_size indices how large the voxels are.
 */
void BoundingBoxServer::voxelizePointCloud(PointCloudPtr point_cloud, PointCloudPtr voxelized_point_cloud, float leaf_size) {
  voxel_grid_.setInputCloud(point_cloud); // Pointer to the input Point Cloud
  voxel_grid_.setLeafSize(leaf_size, leaf_size, leaf_size); // The leaf size of the voxel, each point in this voxel will be indicated by 1 point
  voxel_grid_.filter(*voxelized_point_cloud); // The reduced Point Cloud 
}