#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "stretch_seeing_eye/Feature.h"

#define SIZE 0.2

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

enum rviz_visual_tools::colors COLORS[] = {
    rviz_visual_tools::YELLOW,
    rviz_visual_tools::ORANGE,
    rviz_visual_tools::RED,
};

bool publish_plane(stretch_seeing_eye::Feature::Request &req, stretch_seeing_eye::Feature::Response &res) {
    ROS_DEBUG("Publishing plane");

    if (req.points.size() < 3) {
        for (auto &point : req.points) {
            geometry_msgs::Point p1, p2;
            p1.x = point.x - SIZE;
            p1.y = point.y - SIZE;
            p1.z = 0.0;
            p2.x = point.x + SIZE;
            p2.y = point.y + SIZE;
            p2.z = SIZE;
            visual_tools_->publishCuboid(p1, p2, COLORS[req.detail_level]);
        }
    } else {
        geometry_msgs::Polygon polygon;
        polygon.points = req.points;
        visual_tools_->publishPolygon(polygon, rviz_visual_tools::RED);
    }

    visual_tools_->trigger();

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_plane");
    ros::NodeHandle nh;

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map", "/visualization_marker_array"));
    visual_tools_->loadMarkerPub();

    ros::Duration(1).sleep();

    ros::ServiceServer service = nh.advertiseService("/stretch_seeing_eye/create_marker", publish_plane);

    ros::spin();
    return 0;
}
