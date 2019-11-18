#include <pluginlib/class_list_macros.h>
#include "python_planner/python_planner.h"

PLUGINLIB_EXPORT_CLASS(my_planner::GlobalPlannerPython, nav_core::BaseGlobalPlanner)


namespace my_planner {

GlobalPlannerPython::GlobalPlannerPython() {
  initialized_ = false;
}

GlobalPlannerPython::GlobalPlannerPython(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
  initialized_ = false;
  initialize(name, costmap_ros);
}

void GlobalPlannerPython::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {

  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();

    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();
    resolution_ = costmap_->getResolution();
    map_size_ = width_ * height_;

    makeplan_service_ = private_nh.serviceClient<my_planner::MakePlan>("make_plan");

    makeplan_service_.waitForExistence();

    initialized_ = true;
  }
}

bool GlobalPlannerPython::makePlan(const geometry_msgs::PoseStamped &start,
                                   const geometry_msgs::PoseStamped &goal,
                                   std::vector<geometry_msgs::PoseStamped> &plan) {
  plan.clear(); // just to be sure

  std::vector<int> costmap(map_size_);

  for (size_t idx = 0; idx < map_size_; ++idx) {
    int x, y;
    x = idx % width_;
    y = std::floor(idx / width_);
    costmap.at(idx) = static_cast<int>(costmap_->getCost(x, y));
  }

  // Get the index values of the x and y coordinates
  float start_x = start.pose.position.x;
  float start_y = start.pose.position.y;
  FromPositionToIndex(start_x, start_y);
  size_t start_index = ToIndex(start_x, start_y);

  float goal_x = goal.pose.position.x;
  float goal_y = goal.pose.position.y;
  FromPositionToIndex(goal_x, goal_y);
  size_t goal_index = ToIndex(goal_x, goal_y);

  my_planner::MakePlan makeplan;
  makeplan.request.costmap_ros = costmap;
  makeplan.request.start = start_index;
  makeplan.request.goal = goal_index;
  makeplan.request.width = width_;
  makeplan.request.height = height_;

  makeplan_service_.call(makeplan);
  std::vector<int> index_plan = makeplan.response.plan;

  std::cout << "Number of points: " << index_plan.size() << "\n";

  if (index_plan.size()) {
    index_plan.insert(index_plan.begin(), start_index);
    index_plan.push_back(goal_index);

    for (int p: index_plan) {
      int x, y;
      FromIndex(p, x, y);
      float x_path = static_cast<float>(x);
      float y_path = static_cast<float>(y);

      FromIndexToPosition(x_path, y_path);
      geometry_msgs::PoseStamped position;
      position.header.frame_id = start.header.frame_id;
      position.pose.position.x = x_path;
      position.pose.position.y = y_path;
      position.pose.orientation.x = 0;
      position.pose.orientation.y = 0;
      position.pose.orientation.z = 0;
      position.pose.orientation.w = 1;

      plan.push_back(position);
    }
		
		geometry_msgs::PoseStamped position;
		position.header.frame_id = goal.header.frame_id;
		position.pose = goal.pose;
		plan.push_back(position);

    return true;
  } else {
    return false;
  }
}


size_t GlobalPlannerPython::ToIndex(float x, float y) {
    return y * width_ + x;
  }

void GlobalPlannerPython::FromIndex(size_t index, int &x, int &y) {
  x = index % width_;
  y = std::floor(index / width_);
}

void GlobalPlannerPython::FromPositionToIndex(float &x, float &y) {
  x = static_cast<size_t>((x - origin_x_) / resolution_);
  y = static_cast<size_t>((y - origin_y_) / resolution_);
}

void GlobalPlannerPython::FromIndexToPosition(float &x, float &y) {
    x = x * resolution_ + origin_x_;
    y = y * resolution_ + origin_y_;
  }



}
