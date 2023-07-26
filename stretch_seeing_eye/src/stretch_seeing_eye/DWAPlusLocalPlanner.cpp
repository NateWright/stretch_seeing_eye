#include "DWAPlusLocalPlanner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::DWAPlusPlannerROS, dwa_local_planner::DWAPlannerROS)
PLUGINLIB_EXPORT_CLASS(local_planner::DWAPlusPlannerROS, nav_core::BaseLocalPlanner)

namespace local_planner {

double distance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2) {
    return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2) +
                pow(p1.pose.position.z - p2.pose.position.z, 2));
}

DWAPlusPlannerROS::DWAPlusPlannerROS() : DWAPlannerROS() {}
DWAPlusPlannerROS::~DWAPlusPlannerROS() {}
void DWAPlusPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    DWAPlannerROS::initialize(name, tf, costmap_ros);
}
bool DWAPlusPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    rotate_ = true;
    plan_ = orig_global_plan;
    ROS_INFO_STREAM("New Plan");
    return DWAPlannerROS::setPlan(orig_global_plan);
}
bool DWAPlusPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    if (rotate_) {
        double yaw = calculateYaw(robot_pose);
        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        ROS_INFO_STREAM("Yaw: " << fabs(yaw - robot_yaw));
        if (fabs(yaw - robot_yaw) < 0.1) {
            rotate_ = false;
        } else {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0.1;
            return true;
        }
    }

    DWAPlannerROS::computeVelocityCommands(cmd_vel);
    return true;
}
bool DWAPlusPlannerROS::isGoalReached() {
    return DWAPlannerROS::isGoalReached();
}
double DWAPlusPlannerROS::calculateYaw(geometry_msgs::PoseStamped robotPose) {
    double angle = 0;
    double minDistance = 1000000;
    size_t index = 0;
    for (size_t i = 0; i < plan_.size(); i++) {
        double distance = sqrt(pow(robotPose.pose.position.x - plan_[i].pose.position.x, 2) +
                               pow(robotPose.pose.position.y - plan_[i].pose.position.y, 2));
        if (distance < minDistance) {
            minDistance = distance;
            index = i;
        } else if (distance > minDistance) {
            break;
        }
    }
    geometry_msgs::PoseStamped goalPose = plan_[index];
    size_t count = 1;
    for (size_t i = index + 1; i < plan_.size() && i < index + 5; i++) {
        goalPose.pose.position.x += plan_[i].pose.position.x;
        goalPose.pose.position.y += plan_[i].pose.position.y;
        count++;
    }
    goalPose.pose.position.x /= count;
    goalPose.pose.position.y /= count;
    return atan2(goalPose.pose.position.y - robotPose.pose.position.y,
                 goalPose.pose.position.x - robotPose.pose.position.x);
}

}  // namespace local_planner