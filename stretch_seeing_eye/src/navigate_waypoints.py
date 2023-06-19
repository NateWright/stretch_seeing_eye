import rospy
import math

from std_msgs.msg import Header, String, Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalID
from dynamic_reconfigure import client
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

from stretch_seeing_eye.shortest_path import Graph
from stretch_seeing_eye.waypoint import Waypoint
from stretch_seeing_eye.feature import Feature, DetailLevel
from stretch_seeing_eye.srv import Waypoint as WaypointSrv, WaypointRequest, WaypointResponse, GetWaypoints, GetWaypointsRequest, GetWaypointsResponse


class NavigateWaypoint:
    def __init__(self):
        # Publishers and Subscribers
        self.move_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        self.waypoint_rviz_pub = rospy.Publisher(
            '/visualization_marker_array', MarkerArray, queue_size=10)
        self.move_base_cancel_pub = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=1)

        self.move_base_sub = rospy.Subscriber(
            '/move_base/status', GoalStatusArray, self.move_base_status_callback, queue_size=1)
        self.set_curr_waypoint_sub = rospy.Subscriber(
            '/stretch_seeing_eye/set_curr_waypoint', String, self.set_curr_waypoint_callback, queue_size=1)
        self.set_max_vel_sub = rospy.Subscriber(
            '/stretch_seeing_eye/set_max_vel', Float32, self.set_max_vel_callback, queue_size=1)

        self.navigate_to_waypoint_service = rospy.Service(
            '/stretch_seeing_eye/navigate_to_waypoint', WaypointSrv, self.navigate_to_waypoint)
        self.get_waypoints_service = rospy.Service(
            '/stretch_seeing_eye/get_waypoints', GetWaypoints, self.get_waypoints_callback)
        self.pause_navigation_service = rospy.Service(
            '/stretch_seeing_eye/pause_navigation', SetBool, self.pause_navigation_callback)

        self.client = client.Client(
            '/move_base/DWAPlannerROS', timeout=30, config_callback=None)
        rospy.sleep(1)

        self.moving = False
        self.pause = False
        self.max_val = 0.3

        self.waypoints = {}
        self.features = {}
        self.lookup_table = {}
        self.lookup_table_reverse = {}
        self.curr_feature = 'base'
        self.curr_goal = None
        self.import_data(rospy.get_param('/description_file'))

    def set_max_vel_callback(self, msg: Float32):
        self.max_val = msg.data
        self.update_move_base('max_vel_x', self.max_val)

    def move_base_status_callback(self, msg: GoalStatusArray):
        if len(msg.status_list) and msg.status_list[-1].status == 1:
            self.moving = True
        else:
            self.moving = False
        # rospy.logdebug('Moving: ' + str(self.moving))

    def pause_navigation_callback(self, msg: SetBoolRequest):
        self.pause = msg.data
        if self.pause:
            rospy.logdebug('Pausing navigation')
            self.move_base_cancel_pub.publish(GoalID())
            return SetBoolResponse(True, 'Paused navigation')
        else:
            rospy.logdebug('Resuming navigation')
            self.navigate(self.curr_goal)
            rospy.sleep(1)
            return SetBoolResponse(True, 'Resumed navigation')

    def set_curr_waypoint_callback(self, msg: String):
        self.curr_feature = msg.data.lower()

    def import_data(self, file: str):
        connections = {}
        count = 0

        markers = []
        with open(file, 'r') as f:
            data = f.read().split('---\n')
            for line in data[0].split('\n'):
                if line == "":
                    continue
                w = Waypoint(line)
                self.waypoints[w.name] = w
                markers.append(self.create_marker(
                    self.waypoints[w.name].poseStamped.pose.position, self.waypoints[w.name].poseStamped.pose.orientation, count))
                connections[w.name] = w.connections
                self.lookup_table[w.name] = count
                self.lookup_table_reverse[count] = w.name
                if w.navigatable:
                    self.features[w.name.lower()] = {
                        'name': w.name, 'waypoint': w.name}
                if w.door:
                    self.features[w.name.lower()] = {
                        'name': w.name, 'waypoint': w.name}
                count += 1
            for line in data[1].split('\n'):
                if line == "":
                    continue
                f = Feature(line.strip())
                if f.waypoint is not None:
                    # rospy.logdebug('Feature ' + f.name + ' has waypoint ' + f.waypoint)
                    self.features[f.name.lower()] = {
                        'name': f.name, 'waypoint': f.waypoint}
        self.waypoint_rviz_pub.publish(MarkerArray(markers=markers))
        rows, cols = count, count
        adjacency_matrix = [[0 for i in range(cols)] for j in range(rows)]
        for key, val in connections.items():
            curr_waypoint = self.waypoints[key].poseStamped.pose.position
            for connection in val:
                connection_waypoint = self.waypoints[connection].poseStamped.pose.position
                adjacency_matrix[self.lookup_table[key]][self.lookup_table[connection]] = math.sqrt(
                    (curr_waypoint.x - connection_waypoint.x)**2 + (curr_waypoint.y - connection_waypoint.y)**2)
        self.graph = Graph(count)
        self.graph.graph = adjacency_matrix

    def navigate(self, waypoint: str = 'Base'):
        rospy.logdebug('Navigating to ' + waypoint)
        rospy.logdebug(self.waypoints[waypoint])

        self.curr_goal = waypoint
        self.move_pub.publish(self.waypoints[waypoint].poseStamped)
        rospy.logdebug('Published')

    def navigate_to_waypoint(self, msg: WaypointRequest):
        goal = msg.data.lower()
        if goal not in self.features.keys():
            rospy.logdebug('Feature unknown')
            return WaypointResponse()

        start = self.lookup_table[self.features[self.curr_feature]['waypoint']]
        end = self.lookup_table[self.features[goal]['waypoint']]
        rospy.logdebug('Start: ' + str(start) + ' End: ' + str(end))
        waypoints = list(
            map(lambda x: self.lookup_table_reverse[x], self.graph.dijkstra(start, end)))
        waypoints.append(self.features[goal]['waypoint'])
        rospy.logdebug(waypoints)
        self.update_move_base('max_vel_x', self.max_val)
        self.update_move_base('xy_goal_tolerance', 1.0)
        for i, waypoint in enumerate(waypoints):
            if i == len(waypoints) - 1:
                self.update_move_base('xy_goal_tolerance', 0.1)
            self.navigate(waypoint)
            rospy.sleep(1)
            while self.moving or self.pause:
                rospy.sleep(1)
        self.curr_feature = goal
        rospy.logdebug('Reached ' + goal)
        return WaypointResponse()

    def update_move_base(self, key: str, value: any):
        self.client.update_configuration({key: value})

    def create_marker(self, p: Point, q: Quaternion, count: int):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'test'
        m.id = count
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = Pose(p, q)
        m.scale = Point(0.5, 0.5, 0.5)
        m.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        return m

    def get_waypoints_callback(self, req: GetWaypointsRequest):
        arr = [self.features[key]['name'] for key in self.features.keys()]
        arr.sort()
        return GetWaypointsResponse(waypoints=arr)


if __name__ == '__main__':
    rospy.init_node('navigate_waypoints', log_level=rospy.DEBUG)
    node = NavigateWaypoint()
    rospy.sleep(1)
    # node.navigate_to_waypoint('M1')
    rospy.loginfo('Ready')
    while not rospy.is_shutdown():
        rospy.spin()
