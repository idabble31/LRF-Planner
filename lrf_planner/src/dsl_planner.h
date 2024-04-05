#ifndef D_STAR_LITE_GLOBAL_PLANNER_H
#define D_STAR_LITE_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <dstar_lite/dstar_lite.h>

namespace dstar_lite_global_planner {

class DStarLiteGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
  DStarLiteGlobalPlanner();
  DStarLiteGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

private:
  // D Star Lite related variables and functions
  dstar_lite::DStarLite dstar;
  costmap_2d::Costmap2D* costmap_;
  std::string global_frame_;

  // Helper function to convert a PoseStamped to a 2D position
  bool poseTo2D(const geometry_msgs::PoseStamped& pose, double& x, double& y);
  // Helper function to convert a 2D position to a PoseStamped
  bool posToPose(const double& x, const double& y, geometry_msgs::PoseStamped& pose);
};
}  // namespace dstar_lite_global_planner

#endif  // D_STAR_LITE_GLOBAL_PLANNER_H