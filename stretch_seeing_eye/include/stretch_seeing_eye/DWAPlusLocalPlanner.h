#ifndef DWA_PLUS_LOCAL_PLANNER_H_
#define DWA_PLUS_LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

using namespace std;

namespace local_planner {

class DWAPlusPlannerROS : public dwa_local_planner::DWAPlannerROS {
   public:
    DWAPlusPlannerROS();

    ~DWAPlusPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

   private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    std::vector<geometry_msgs::PoseStamped> plan_;
    bool rotate_;

    double calculateYaw(geometry_msgs::PoseStamped robotPose);
};
};  // namespace local_planner

#endif