import rospy
import math

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalID
from dynamic_reconfigure import client
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32, String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from stretch_seeing_eye.Waypoint import Waypoint
from stretch_seeing_eye.Door import Door
from stretch_seeing_eye.srv import Waypoint as WaypointSrv, WaypointRequest, WaypointResponse, GetWaypoints, GetWaypointsRequest, GetWaypointsResponse
from stretch_seeing_eye.msg import WaypointDijkstra


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
        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.waypoint_rviz_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/move_base/WaypointGlobalPlanner/waypoint', WaypointDijkstra, queue_size=1)  # Publish adjacency matrix
        self.message_pub = rospy.Publisher('/stretch_seeing_eye/message', String, queue_size=10)

        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback, queue_size=1)
        self.set_max_vel_sub = rospy.Subscriber('/stretch_seeing_eye/set_max_vel', Float32, self.set_max_vel_callback, queue_size=1)

        self.navigate_to_waypoint_service = rospy.Service('/stretch_seeing_eye/navigate_to_waypoint', WaypointSrv, self.navigate_to_waypoint)
        self.get_waypoints_service = rospy.Service('/stretch_seeing_eye/get_waypoints', GetWaypoints, self.get_waypoints_callback)
        self.stop_navigation_service = rospy.Service('/stretch_seeing_eye/stop_navigation', Trigger, self.stop_navigation_callback)
        self.check_door_service = rospy.ServiceProxy('/stretch_seeing_eye/detect_door_open', Trigger)

        self.client = client.Client('/move_base/DWAPlannerROS', timeout=30, config_callback=None)
        rospy.sleep(1)

        self.moving = False
        self.pause = False
        self.max_val = 0.3

        self.waypoints = {}
        self.doors = {}
        self.goals = {}

        try:
            self.import_data(rospy.get_param(
                '/description_file'))  # type: ignore
        except Exception as e:
            rospy.logerr(e)
            exit(1)

    def move_base_status_callback(self, msg: GoalStatusArray):
        if len(msg.status_list) and msg.status_list[-1].status == 1:  # type: ignore
            self.moving = True
        else:
            self.moving = False

    def set_max_vel_callback(self, msg: Float32):
        self.max_val = msg.data
        self.update_move_base('max_vel_x', self.max_val)

    def stop_navigation_callback(self, msg: TriggerRequest):
        self.move_base_cancel_pub.publish(GoalID())
        return TriggerResponse(success=True)

    def import_data(self, file: str):
        count = 0

        markers = []
        with open(file, 'r') as f:
            data = f.read()
            for line in data.split('\n'):
                if line == "":
                    continue
                elif line.startswith('Door'):
                    d = Door(line)
                    self.doors[d.id] = d
                    count += 1
                    markers.append(self.create_marker(
                        d.entrance_pose, d.id, ColorRGBA(1.0, 0.0, 0.0, 1.0)))
                    if d.inside:
                        q = quaternion_from_euler(0, 0, math.atan2(d.inside_pose.pose.position.y - d.entrance_pose.pose.position.y, d.inside_pose.pose.position.x - d.entrance_pose.pose.position.x))
                        d.entrance_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                        q = quaternion_from_euler(0, 0, math.atan2(d.entrance_pose.pose.position.y - d.inside_pose.pose.position.y, d.entrance_pose.pose.position.x - d.inside_pose.pose.position.x))
                        d.inside_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                        self.goals[d.name + ' Inside'] = d.inside_pose
                        self.goals[d.name + ' Entrance'] = d.entrance_pose
                        count += 1
                        markers.append(self.create_marker(
                            d.inside_pose, -d.id, ColorRGBA(1.0, 0.0, 0.0, 1.0)))
                    else:
                        self.goals[d.name] = d.entrance_pose
                elif line.startswith('Waypoint'):
                    w = Waypoint(line)
                    self.waypoints[w.id] = w
                    count += 1
                    markers.append(self.create_marker(w.poseStamped, w.id))
                    self.goals[w.name] = w.poseStamped
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

    def navigate_to_waypoint(self, msg: WaypointRequest):
        if msg.data is None or msg.data == '':
            return WaypointResponse()

        goal = msg.data
        if goal not in self.goals.keys():
            rospy.logdebug('Goal unknown')
            return WaypointResponse()

        inside = False
        if 'Entrance' in goal:
            self.update_move_base('yaw_goal_tolerance', 0.05)
            goal_pose = self.goals[goal]
            self.message_pub.publish(String(data='Navigating to ' + goal))
        elif 'Inside' in goal:
            self.update_move_base('yaw_goal_tolerance', 0.05)
            inside = True
            goal_pose = self.goals[goal.replace('Inside', 'Entrance')]
            self.message_pub.publish(String(data='Navigating to ' + goal.replace('Inside', 'Entrance') + ' and then through the door if open'))
        else:
            self.update_move_base('yaw_goal_tolerance', 6.28)
            goal_pose = self.goals[goal]
            self.message_pub.publish(String(data='Navigating to ' + goal))

        self.move_base_goal_pub.publish(goal_pose)
        rospy.sleep(2)
        while self.moving:
            rospy.sleep(1)

        if inside:
            # Check for door
            rospy.logdebug('Checking for door')
            req = TriggerRequest()
            res = self.check_door_service(req)
            if res.success:
                rospy.logdebug('Door open')
                self.message_pub.publish(String(data='Proceeding through door'))
                self.move_base_goal_pub.publish(self.goals[goal])
                rospy.sleep(2)
                while self.moving:
                    rospy.sleep(1)
            else:
                rospy.logdebug('Door closed')
                self.message_pub.publish(String(data='Door closed'))

        return WaypointResponse()

    def update_move_base(self, key: str, value: any):  # type: ignore
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
        return GetWaypointsResponse(waypoints=[key for key in self.goals.keys()])


if __name__ == '__main__':
    rospy.init_node('navigate_waypoints', log_level=rospy.DEBUG)
    node = NavigateWaypoint()
    rospy.sleep(1)
    # node.navigate_to_waypoint('M1')
    rospy.loginfo('Ready')
    while not rospy.is_shutdown():
        rospy.spin()
