#include "DWAPlusLocalPlanner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::DWAPlusPlannerROS, dwa_local_planner::DWAPlannerROS)
PLUGINLIB_EXPORT_CLASS(local_planner::DWAPlusPlannerROS, nav_core::BaseLocalPlanner)

namespace local_planner {

DWAPlusPlannerROS::DWAPlusPlannerROS() : DWAPlannerROS() {}
DWAPlusPlannerROS::~DWAPlusPlannerROS() {}
void DWAPlusPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    DWAPlannerROS::initialize(name, tf, costmap_ros);
}
bool DWAPlusPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    return DWAPlannerROS::setPlan(orig_global_plan);
}
bool DWAPlusPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    return DWAPlannerROS::computeVelocityCommands(cmd_vel);
}
bool DWAPlusPlannerROS::isGoalReached() {
    return DWAPlannerROS::isGoalReached();
}

}  // namespace local_planner