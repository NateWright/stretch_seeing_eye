#include <geometry_msgs/PoseStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <iterator>

#include "stretch_seeing_eye/CheckClear.h"

class DoorDetector {
   public:
    DoorDetector(ros::NodeHandlePtr nh) : nh(nh), tfListener(tfBuffer) {
        // cloudSub = nh->subscribe("/test_pointcloud", 1, &DoorDetector::door_detection, this);
        // imageSub = nh->subscribe("/test_image", 1, &DoorDetector::door_detection_image, this);

        checkClearService = nh->advertiseService("/stretch_seeing_eye/detect_door_open", &DoorDetector::door_detection_srv, this);
        getPath = nh->advertiseService("/stretch_seeing_eye/create_path", &DoorDetector::create_path_srv, this);

        costmapSub = nh->subscribe("/move_base/local_costmap/costmap", 1, &DoorDetector::costmap_callback, this);
        costmapUpdateSub = nh->subscribe("/move_base/local_costmap/costmap_updates", 1, &DoorDetector::costmap_update_callback, this);
        debugPath = nh->advertise<nav_msgs::Path>("/stretch_seeing_eye/debug_path", 10);
        debugCostmap = nh->advertise<nav_msgs::OccupancyGrid>("/stretch_seeing_eye/debug_costmap", 10);

        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/rviz_visual_markers"));
    }
    bool door_detection_srv(stretch_seeing_eye::CheckClear::Request &req, stretch_seeing_eye::CheckClear::Response &res) {
        if (!costmap) {
            res.success = false;
            res.message = "No costmap received";
            return true;
        }
        int x1 = (req.p1.pose.position.x - costmap->info.origin.position.x) / costmap->info.resolution,
            y1 = (req.p1.pose.position.y - costmap->info.origin.position.y) / costmap->info.resolution,
            x2 = (req.p2.pose.position.x - costmap->info.origin.position.x) / costmap->info.resolution,
            y2 = (req.p2.pose.position.y - costmap->info.origin.position.y) / costmap->info.resolution;

        int dx = abs(x2 - x1);
        int sx = x1 < x2 ? 1 : -1;
        int dy = -abs(y2 - y1);
        int sy = y1 < y2 ? 1 : -1;
        int err = dx + dy;
        nav_msgs::Path p;
        p.header.frame_id = "map";
        debugCostmap.publish(*costmap);
        while (true) {
            // check x1 and y1
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x1 * costmap->info.resolution + costmap->info.origin.position.x;
            pose.pose.position.y = y1 * costmap->info.resolution + costmap->info.origin.position.y;
            pose.pose.position.z = 0;
            p.poses.push_back(pose);
            if (costmap->data[x1 + y1 * costmap->info.width] >= 90) {
                res.success = false;
                res.message = "Obstacle in the way\nVal: " + std::to_string(costmap->data[x1 + y1 * costmap->info.width]);
                debugPath.publish(p);
                return true;
            }
            if (x1 == x2 && y1 == y2) break;
            int e2 = 2 * err;
            if (e2 >= dy) {
                if (x1 == x2) break;
                err += dy;
                x1 += sx;
            }
            if (e2 <= dx) {
                if (y1 == y2) break;
                err += dx;
                y1 += sy;
            }
        }
        debugPath.publish(p);
        res.success = true;
        res.message = "No obstacle in the way";
        return true;
    }
    bool create_path_srv(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res) {
        if (!costmap) {
            return true;
        }
        int x1 = (req.start.pose.position.x - costmap->info.origin.position.x) / costmap->info.resolution,
            y1 = (req.start.pose.position.y - costmap->info.origin.position.y) / costmap->info.resolution,
            x2 = (req.goal.pose.position.x - costmap->info.origin.position.x) / costmap->info.resolution,
            y2 = (req.goal.pose.position.y - costmap->info.origin.position.y) / costmap->info.resolution;

        int dx = abs(x2 - x1);
        int sx = x1 < x2 ? 1 : -1;
        int dy = -abs(y2 - y1);
        int sy = y1 < y2 ? 1 : -1;
        int err = dx + dy;
        res.plan.header.frame_id = "";
        if (debugCostmap.getNumSubscribers() > 0)
            debugCostmap.publish(*costmap);
        while (true) {
            // check x1 and y1
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x1 * costmap->info.resolution + costmap->info.origin.position.x;
            pose.pose.position.y = y1 * costmap->info.resolution + costmap->info.origin.position.y;
            pose.pose.position.z = 0;
            res.plan.poses.push_back(pose);
            if (costmap->data[x1 + y1 * costmap->info.width] >= 90) {
                if (debugPath.getNumSubscribers() > 0)
                    debugPath.publish(res.plan);
                return true;
            }
            if (x1 == x2 && y1 == y2) break;
            int e2 = 2 * err;
            if (e2 >= dy) {
                if (x1 == x2) break;
                err += dy;
                x1 += sx;
            }
            if (e2 <= dx) {
                if (y1 == y2) break;
                err += dx;
                y1 += sy;
            }
        }
        if (debugPath.getNumSubscribers() > 0)
            debugPath.publish(res.plan);
        return true;
    }
    void costmap_callback(const nav_msgs::OccupancyGrid::Ptr msg) {
        costmap = msg;
        // ROS_INFO_STREAM("Costmap received");
        // ROS_INFO_STREAM("Costmap width: " << costmap->info.width);
        // ROS_INFO_STREAM("Costmap height: " << costmap->info.height);
    }
    void costmap_update_callback(const map_msgs::OccupancyGridUpdate::Ptr msg) {
        if (!costmap) return;
        // ROS_INFO_STREAM("Costmap update received");
        // ROS_INFO_STREAM("Costmap update width: " << msg->width);
        // ROS_INFO_STREAM("Costmap update height: " << msg->height);

        for (auto x = msg->x; x < msg->x + msg->width; x++) {
            for (auto y = msg->y; y < msg->y + msg->height; y++) {
                costmap->data[x + y * costmap->info.width] = msg->data[(x - msg->x) + (y - msg->y) * msg->width];
            }
        }
        // costmap->data = msg->data;
    }

   private:
    ros::NodeHandlePtr nh;

    // srvs
    ros::ServiceServer checkClearService;
    ros::ServiceServer getPath;

    // Debug Publishers
    ros::Publisher debugPath;
    ros::Publisher debugCostmap;

    // Costmap updates
    ros::Subscriber costmapSub;
    ros::Subscriber costmapUpdateSub;
    nav_msgs::OccupancyGrid::Ptr costmap;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::string message;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_door");
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    DoorDetector doorDetector(nh);
    ros::spin();
    return 0;
}
