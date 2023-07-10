import rospy
import math
import actionlib

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, String, Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from actionlib_msgs.msg import GoalStatusArray, GoalID
from dynamic_reconfigure import client
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from mbf_msgs.msg import GetPathAction, GetPathGoal, GetPathResult, ExePathAction, ExePathGoal, ExePathFeedback
from nav_msgs.msg import Path

from stretch_seeing_eye.shortest_path import Graph
from stretch_seeing_eye.Waypoint import Waypoint
from stretch_seeing_eye.Door import Door
from stretch_seeing_eye.feature import Feature, DetailLevel
from stretch_seeing_eye.srv import Waypoint as WaypointSrv, WaypointRequest, WaypointResponse, GetWaypoints, GetWaypointsRequest, GetWaypointsResponse
from stretch_seeing_eye.msg import WaypointDijkstra

# class MoveBaseWaypointClient:
#     def __init__(self):
#         self.pathClient = actionlib.SimpleActionClient(
#             '/mbf_costmap_nav/get_path', GetPathAction)
#         self.exePathClient = actionlib.SimpleActionClient(
#             '/mbf_costmap_nav/exe_path', ExePathAction)

#         self.path_complete = False
#         self.paths = []
#         self.pub = rospy.Publisher(
#             '/test_path', Path, queue_size=10)
#         self.pub2 = rospy.Publisher(
#             '/test_pose', PoseArray, queue_size=10)
#         self.pathClient.wait_for_server()
#         rospy.logdebug('connected to mbf server')

#     def navigate_waypoints(self, points: list, angle=None):
#         self.points = points
#         self.path_complete = False
#         self.paths = []
#         trajectory = ExePathGoal()
#         trajectory.path.header.frame_id = 'map'
#         trajectory.path.poses = []
#         trajectory.tolerance_from_action = True
#         trajectory.dist_tolerance = 0.1
#         trajectory.angle_tolerance = 6.28

#         # Get path
#         for i, point in enumerate(points):
#             goal = GetPathGoal()
#             if i == 0:
#                 goal.use_start_pose = False
#                 goal.target_pose = point
#                 self.pathClient.send_goal(goal)
#                 self.pathClient.wait_for_result()
#                 result = self.pathClient.get_result()
#                 if result is not None:
#                     trajectory.path.poses = result.path.poses
#                     # trajectory.path.poses.pop()
#                     self.paths.append(
#                         self.calculateLength(trajectory.path.poses))
#                 continue
#             goal.use_start_pose = True
#             goal.start_pose = trajectory.path.poses[-1]
#             goal.target_pose = point
#             self.pathClient.send_goal(goal)
#             self.pathClient.wait_for_result()
#             result = self.pathClient.get_result()
#             if result is not None:
#                 # result.path.poses.pop()
#                 self.paths.append(self.calculateLength(result.path.poses))
#                 trajectory.path.poses.extend(result.path.poses)
#         # end path
#         # Debug path
#         for i, pose in enumerate(trajectory.path.poses):
#             pose.pose.orientation = Quaternion(0, 0, 0, 1)
#         if angle is not None:
#             trajectory.path.poses[-1].pose.orientation = Quaternion(
#                 *quaternion_from_euler(0, 0, angle))
#         self.pub.publish(trajectory.path)
#         self.pub2.publish(PoseArray(header=Header(frame_id='map'), poses=[
#                           p.pose for p in trajectory.path.poses]))
#         self.paths.reverse()
#         for i, length in enumerate(self.paths):
#             if i == 0:
#                 continue
#             self.paths[i] += self.paths[i-1]
#         rospy.logdebug(self.paths)
#         self.exePathClient.send_goal(
#             trajectory, done_cb=self.done, feedback_cb=self.feedback_cb)

#     def feedback_cb(self, feedback: ExePathFeedback):
#         if len(self.paths) > 0 and feedback.dist_to_goal < self.paths[-1]:
#             self.paths.pop()
#             self.points = self.points[1:]

#     def done(self, status, result):
#         self.path_complete = True

#     def pause(self):
#         self.exePathClient.cancel_goal()

#     def resume(self):
#         self.navigate_waypoints(self.points)

#     def calculateLength(self, path):
#         length = 0
#         for i in range(1, len(path)):
#             length += math.sqrt((path[i].pose.position.x - path[i-1].pose.position.x)**2 +
#                                 (path[i].pose.position.y - path[i-1].pose.position.y)**2)
#         return length


def get_PoseStamped(data):
    if isinstance(data, Waypoint):
        return data.poseStamped
    elif isinstance(data, Door):
        return data.entrance_pose
    return None


def distance(p1: PoseStamped, p2: PoseStamped) -> float:
    return math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 +
                     (p1.pose.position.y - p2.pose.position.y)**2)


class NavigateWaypoint:
    def __init__(self):
        # Publishers and Subscribers
        # self.test_client = MoveBaseWaypointClient()
        self.move_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        self.waypoint_rviz_pub = rospy.Publisher(
            '/visualization_marker_array', MarkerArray, queue_size=10)
        self.move_base_cancel_pub = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=1)
        self.waypoint_pub = rospy.Publisher(
            '/move_base/WaypointGlobalPlanner/waypoint', WaypointDijkstra, queue_size=1)

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

        # self.client = client.Client(
        #     '/mbf_costmap_nav/DWAPlannerROS', timeout=30, config_callback=None)
        rospy.sleep(1)

        self.moving = False
        self.pause = False
        self.max_val = 0.3

        self.waypoints = {}
        self.doors = {}
        self.features = {}
        self.curr_feature = 'Base'
        self.curr_goal = None
        try:
            self.import_data(rospy.get_param(
                '/description_file'))  # type: ignore
        except Exception as e:
            rospy.logerr(e)
            exit(1)

    def set_max_vel_callback(self, msg: Float32):
        self.max_val = msg.data
        self.update_move_base('max_vel_x', self.max_val)

    def pause_navigation_callback(self, msg: SetBoolRequest):
        self.pause = msg.data
        if self.pause:
            self.test_client.pause()
            return SetBoolResponse(True, 'Paused navigation')
        else:
            rospy.logdebug('Resuming navigation')
            self.test_client.resume()
            return SetBoolResponse(True, 'Resumed navigation')

    def set_curr_waypoint_callback(self, msg: String):
        self.curr_feature = msg.data

    def import_data(self, file: str):
        count = 0

        markers = []
        with open(file, 'r') as f:
            data = f.read()
            for line in data.split('\n'):
                rospy.logdebug(line)
                if line == "":
                    continue
                elif line.startswith('Door'):
                    d = Door(line)
                    self.doors[d.id] = d
                    count += 1
                    markers.append(self.create_marker(
                        d.entrance_pose, d.id, ColorRGBA(1.0, 0.0, 0.0, 1.0)))
                    if d.inside:
                        count += 1
                        markers.append(self.create_marker(
                            d.inside_pose, d.id, ColorRGBA(1.0, 0.0, 0.0, 1.0)))
                elif line.startswith('Waypoint'):
                    w = Waypoint(line)
                    self.waypoints[w.id] = w
                    count += 1
                    markers.append(self.create_marker(w.poseStamped, w.id))
        self.waypoint_rviz_pub.publish(MarkerArray(markers=markers))

        # Create graph
        adjacency_matrix = [[0 for i in range(count)] for j in range(count)]
        points: list[PoseStamped] = []
        lookup_table = {}  # id -> index
        i = 0
        for key, val in self.doors.items():
            lookup_table[key] = i
            points.append(val.entrance_pose)
            i += 1
            if val.inside:
                points.append(val.inside_pose)
                adjacency_matrix[i - 1][i] = distance(val.entrance_pose, val.inside_pose)  # type: ignore
                adjacency_matrix[i][i - 1] = adjacency_matrix[i - 1][i]
                i += 1

        for key, val in self.waypoints.items():
            lookup_table[val.id] = i
            points.append(val.poseStamped)
            i += 1
            for c in val.connections:
                if c in lookup_table.keys():
                    j = lookup_table[c]
                    p = points[j]
                    adjacency_matrix[i - 1][j] = distance(points[-1], p)  # type: ignore
                    adjacency_matrix[j][i - 1] = adjacency_matrix[i - 1][j]
            for d in val.doors:
                if d in lookup_table.keys():
                    j = lookup_table[d]
                    p = points[j]
                    adjacency_matrix[i - 1][j] = distance(points[-1], p)  # type: ignore
                    adjacency_matrix[j][i - 1] = adjacency_matrix[i - 1][j]
        msg = WaypointDijkstra()
        msg.waypoints = points
        msg.graph = [Float32(data=v) for row in adjacency_matrix for v in row]
        self.waypoint_pub.publish(msg)

    def navigate(self, waypoint: str = 'Base'):
        rospy.logdebug('Navigating to ' + waypoint)
        rospy.logdebug(self.waypoints[waypoint])

        self.curr_goal = waypoint
        self.move_pub.publish(self.waypoints[waypoint].poseStamped)
        rospy.logdebug('Published')

    def navigate_to_waypoint(self, msg: WaypointRequest):
        goal = msg.data
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

        door = False
        inside = False

        if 'Entrance' in goal:
            door = True

        elif 'Inside' in goal:
            door = True
            inside = True

        points = [self.waypoints[w].poseStamped for w in waypoints]
        if inside:
            points.pop()
        self.test_client.navigate_waypoints(points)
        while not self.test_client.path_complete:
            rospy.sleep(0.1)
        if door:
            door_open = True
            rospy.logdebug('Door')
            if door_open and inside:
                self.test_client.navigate_waypoints(
                    [self.waypoints[waypoints[-1].poseStamped]])
        self.curr_feature = goal
        rospy.logdebug('Reached ' + goal)
        return WaypointResponse()

    def update_move_base(self, key: str, value: any):
        self.client.update_configuration({key: value})

    def create_marker(self, poseStamped: PoseStamped, count: int, color: ColorRGBA = ColorRGBA(0.0, 1.0, 0.0, 1.0)):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'test'
        m.id = count
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = poseStamped.pose
        m.scale = Point(0.5, 0.5, 0.5)  # type: ignore
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
