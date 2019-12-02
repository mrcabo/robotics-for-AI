#include "bounding_box_server/bounding_box_server.h"

//! Constructor.
/** Get the point_cloud_topic giving via a launch files, otherwise uses default "/front_xtion/depth_registered/points".
 * Creates the services and the timer for running the bounding box server.
 * The timer is disabled by default and needs to be enabled via a service call.
 * transform_to_link_ is hardcoded ro "/root", this will also be the arms planning frame of reference
 * \param nh a ros::NodeHandle created in the main function.  
 */ 
BoundingBoxServer::BoundingBoxServer(ros::NodeHandle &nh) :
    nh_(nh),
    bounding_box_publisher_(nh_),
    transform_to_link_("/base_link") {

  ros::param::param(std::string("/point_cloud_topic"), point_cloud_topic_, std::string("xtion/depth_registered/points"));

  enable_bounding_box_publisher_service_ = nh_.advertiseService("/enable_bounding_box_publisher", &BoundingBoxServer::enableBoundingBoxPublisher, this);
  disable_bounding_box_publisher_service_ = nh_.advertiseService("/disable_bounding_box_publisher", &BoundingBoxServer::disableBoundingBoxPublisher, this);

  // Create timer to run at 2Hz, timers is disabled in startup 
  update_timer_ = nh_.createTimer(ros::Duration(0.5), &BoundingBoxServer::updateTimerEvent, this, false, false);
}

//! Service callback to start computing and publishing the bounding boxes.
/** Service call arguments are empty.
 *  \return true.
 */
bool BoundingBoxServer::enableBoundingBoxPublisher(std_srvs::EmptyRequest &, std_srvs::EmptyResponse &) {
  
  if (!update_timer_.hasStarted())  { // Check if timer is running or not, if not then start
    update_timer_.start();
  }

  return true;
}

//! Service callback to stop computing and publising the bounding boxes.
/** Service call arguments are empty.
 * \return true.
 */
bool BoundingBoxServer::disableBoundingBoxPublisher(std_srvs::EmptyRequest &, std_srvs::EmptyResponse &) {
   
  if (update_timer_.hasStarted()) { // If timer is still running, stop
    update_timer_.stop();
  }

  return true;
}
