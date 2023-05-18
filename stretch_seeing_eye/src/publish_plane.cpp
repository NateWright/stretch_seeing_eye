#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "stretch_seeing_eye/Plane.h"

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

bool publish_plane(stretch_seeing_eye::Plane::Request &req, stretch_seeing_eye::Plane::Response &res) {
    ROS_DEBUG("Publishing plane");
    geometry_msgs::Point p1, p2;
    p1.x = req.x1;
    p1.y = req.y1;
    p1.z = 0;
    p2.x = req.x2;
    p2.y = req.y2;
    p2.z = 0.05;
    visual_tools_->publishCuboid(p1, p2, rviz_visual_tools::RED);
    visual_tools_->trigger();

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_plane");
    ros::NodeHandle nh;

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "/rviz_visual_markers"));
    visual_tools_->loadMarkerPub();

    ros::Duration(1).sleep();

    ros::ServiceServer service = nh.advertiseService("/stretch_seeing_eye/create_plane_marker", publish_plane);

    ros::spin();
    return 0;
}
