/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <iostream>
#include <vector>

#include "Dijkstra.h"
#include "stretch_seeing_eye/WaypointDijkstra.h"

using std::string;

#ifndef WAYPOINT_GLOBAL_PLANNER_CPP
#define WAYPOINT_GLOBAL_PLANNER_CPP

namespace waypoint_global_planner {

class WaypointGlobalPlanner : public global_planner::GlobalPlanner {
   public:
    WaypointGlobalPlanner();
    WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap_ros, std::string frame_id);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    void waypointCallback(const stretch_seeing_eye::WaypointDijkstraPtr& msg);
    size_t closestWaypoint(const geometry_msgs::PoseStamped& pose);

   private:
    ros::NodeHandle pnh;
    ros::Subscriber waypoint_sub;
    ros::ServiceServer waypoint_goal_srv;
    std::vector<geometry_msgs::PoseStamped> waypoints;
    Dijkstra dijkstra;
};

double pathDistance(const std::vector<geometry_msgs::PoseStamped>& path);
};  // namespace waypoint_global_planner
#endif