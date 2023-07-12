#include "WaypointGlobalPlanner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner, global_planner::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)

using std::vector;
typedef std::vector<geometry_msgs::PoseStamped> path;

namespace waypoint_global_planner {
WaypointGlobalPlanner::WaypointGlobalPlanner() : GlobalPlanner() {
}
WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap_ros, std::string frame_id) : GlobalPlanner() {
    initialize(name, costmap_ros, frame_id);
}

void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}
void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    ROS_INFO_STREAM("Initializing WaypointGlobalPlanner");
    GlobalPlanner::initialize(name, costmap, frame_id);
    dijkstra = Dijkstra(Graph());
    pnh = ros::NodeHandle("~/" + name);
    waypoint_sub = pnh.subscribe("waypoint", 1, &WaypointGlobalPlanner::waypointCallback, this);
}

bool WaypointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                     const geometry_msgs::PoseStamped& goal,
                                     std::vector<geometry_msgs::PoseStamped>& plan) {
    if (waypoints.empty()) {
        return false;
    }
    size_t current_waypoint = closestWaypoint(start);
    size_t goal_waypoint = closestWaypoint(goal);
    if (current_waypoint == goal_waypoint) {
        plan.push_back(start);
        plan.push_back(goal);
        return true;
    }
    vector<int> path = dijkstra.DijkstraAlgo(current_waypoint, goal_waypoint);
    path.push_back(goal_waypoint);
    std::vector<geometry_msgs::PoseStamped> points;
    std::vector<geometry_msgs::PoseStamped> tempPath;
    for (size_t i = 0; i < path.size(); i++) {
        points.push_back(waypoints[path[i]]);
    }
    for (size_t i = 0; i < path.size() - 1; i++) {
        GlobalPlanner::makePlan(points[i], points[i + 1], tempPath);
        plan.insert(plan.end(), tempPath.begin(), tempPath.end());
    }
    plan.back().pose.orientation = goal.pose.orientation;
    nav_msgs::Path plan_msg;
    plan_msg.header.frame_id = frame_id_;
    plan_msg.header.stamp = ros::Time::now();
    plan_msg.poses = plan;
    plan_pub_.publish(plan_msg);
    return true;
}

void WaypointGlobalPlanner::waypointCallback(const stretch_seeing_eye::WaypointDijkstraPtr& msg) {
    waypoints = msg->waypoints;
    const size_t num_waypoints = waypoints.size();
    Graph graph(num_waypoints, std::vector<float>(num_waypoints, 0));
    for (size_t i = 0; i < num_waypoints; i++) {
        for (size_t j = 0; j < num_waypoints; j++) {
            graph[i][j] = msg->graph[i * num_waypoints + j].data;
        }
        // graph[i].insert(graph[i].end(), msg->graph.begin() + i * num_waypoints, msg->graph.begin() + (i + 1) * num_waypoints);
    }
    dijkstra = Dijkstra(graph);
}
size_t WaypointGlobalPlanner::closestWaypoint(const geometry_msgs::PoseStamped& pose) {
    size_t closest_waypoint = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < waypoints.size(); i++) {
        path plan;
        if (GlobalPlanner::makePlan(pose, waypoints[i], plan)) {
            double dist = pathDistance(plan);
            if (dist < min_dist) {
                min_dist = dist;
                closest_waypoint = i;
            }
        }
    }
    return closest_waypoint;
}

double pathDistance(const path& path) {
    double distance = 0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        distance += std::hypot(path[i].pose.position.x - path[i + 1].pose.position.x, path[i].pose.position.y - path[i + 1].pose.position.y);
    }
    return distance;
}

};  // namespace waypoint_global_planner