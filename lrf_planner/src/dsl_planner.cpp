#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <dstar_lite/dstar_lite.h>
#include "d_star_lite_global_planner.h"

namespace dstar_lite_global_planner {

DStarLiteGlobalPlanner::DStarLiteGlobalPlanner() : dstar(100, 100) {}

DStarLiteGlobalPlanner::DStarLiteGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : BaseGlobalPlanner(name, costmap_ros), costmap_(costmap_ros->getCostmap()) {
  initialize(name, costmap_ros);
}

void DStarLiteGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  if (!costmap_) {
    ROS_ERROR("Error getting costmap");
    return;
  }

  if (costmap_->getCost(0, 0) == costmap_2d::NO_INFORMATION) {
    ROS_WARN("Costmap does not contain any information");
  }

  dstar.initialize(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
}

bool DStarLiteGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan) {
  ROS_DEBUG("Generating global plan using D* Lite");

  double start_x, start_y, goal_x, goal_y;

  if (!poseTo2D(start, start_x, start_y) || !poseTo2D(goal, goal_x, goal_y)) {
    ROS_ERROR("Error converting start or goal pose to 2D position");
    return false;
  }

  // Set up D* Lite with the costmap
  dstar.setStart(start_x, start_y, costmap_->getCost(start_x, start_y));
  dstar.setGoal(goal_x, goal_y, costmap_->getCost(goal_x, goal_y));
  dstar.compute();

  // Get the plan from D* Lite
  std::vector<dstar_lite::Vertex> dstar_plan;
  dstar.getPlan(dstar_plan);

  // Convert the D*Lite plan to a PoseStamped plan
  plan.clear();
  for (size_t i = 0; i < dstar_plan.size(); ++i) {
    dstar_lite::Vertex v = dstar_plan[i];
    geometry_msgs::PoseStamped pose;
    if (posToPose(v.x, v.y, pose)) {
      plan.push_back(pose);
    }
  }

  return true;
}

bool DStarLiteGlobalPlanner::poseTo2D(const geometry_msgs::PoseStamped& pose, double& x, double& y) {
  costmap_2d::World* world = costmap_->getWorld();
  if (world == nullptr) {
    ROS_ERROR("Error getting costmap world");
    return false;
  }

  costmap_2d::Point p;
  bool success = world->projectVectorToMap(tf::resolve(global_frame_, pose.header.frame_id),pose.pose, p);
  if (!success) {
    ROS_ERROR("Error projecting pose to map");
    return false;
  }

  x = p.x;
  y = p.y;
  return true;
}

bool DStarLiteGlobalPlanner::posToPose(const double& x, const double& y, geometry_msgs::PoseStamped& pose) {
  costmap_2d::Costmap2D* costmap = costmap_;
  if (costmap == nullptr) {
    ROS_ERROR("Error getting costmap");
    return false;
  }

  costmap_2d::Point p;
  p.x = x;
  p.y = y;

  if (!costmap->worldToMap(p, p)) {
    ROS_ERROR("Error converting 2D position to pose");
    return false;
  }

  pose.header.frame_id = global_frame_;
  pose.pose.position.x = p.x;
  pose.pose.position.y = p.y;
  pose.pose.orientation.w = p.w;