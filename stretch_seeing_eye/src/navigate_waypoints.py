import rospy
import math
import tf2_ros

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalID
from dynamic_reconfigure import client
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32, String, UInt32
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from stretch_seeing_eye.Waypoint import Waypoint
from stretch_seeing_eye.Door import Door
from stretch_seeing_eye.srv import Waypoint as WaypointSrv, WaypointRequest, WaypointResponse
from stretch_seeing_eye.srv import GetWaypoints, GetWaypointsRequest, GetWaypointsResponse
from stretch_seeing_eye.srv import CheckClear, CheckClearRequest, CheckClearResponse
from stretch_seeing_eye.srv import MakePath, MakePathRequest, MakePathResponse
from stretch_seeing_eye.msg import WaypointDijkstra, Door as DoorMsg


def distance(p1: PoseStamped, p2: PoseStamped) -> float:
    return math.hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y)


class NavigateWaypoint:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.moving = False
        self.pause = False
        self.cancel = False
        self.max_vel = 0.3
        self.scaling_factor = 1.0

        self.waypoints = {}  # id -> Waypoint
        self.doors = {}  # id -> Door
        self.goals = {}  # name -> Waypoint/Door
        self.nav_waypoints: list[str] = []

        # Publishers and Subscribers
        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.waypoint_rviz_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10, latch=True)
        self.waypoint_pub = rospy.Publisher('/move_base/WaypointGlobalPlanner/waypoint', WaypointDijkstra, queue_size=1, latch=True)  # Publish adjacency matrix
        self.current_waypoint_pub = rospy.Publisher('/move_base/WaypointGlobalPlanner/current_waypoint', UInt32, queue_size=1, latch=True)  # Publish adjacency matrix
        self.message_pub = rospy.Publisher('/stretch_seeing_eye/message', String, queue_size=10)
        self.door_msg_pub = rospy.Publisher('/stretch_seeing_eye/door', DoorMsg, queue_size=10)

        self.move_base_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback, queue_size=1)
        self.set_max_vel_sub = rospy.Subscriber('/stretch_seeing_eye/set_max_vel', Float32, self.set_max_vel_callback, queue_size=1)
        self.handle_sub = rospy.Subscriber('/stretch_seeing_eye/handle_reading', UInt32, self.handle_callback, queue_size=1)

        self.navigate_to_waypoint_service = rospy.Service('/stretch_seeing_eye/navigate_to_waypoint', WaypointSrv, self.navigate_to_waypoint)
        self.get_waypoints_service = rospy.Service('/stretch_seeing_eye/get_waypoints', GetWaypoints, self.get_waypoints_callback)
        self.stop_navigation_service = rospy.Service('/stretch_seeing_eye/stop_navigation', Trigger, self.stop_navigation_callback)

        self.check_door_service = rospy.ServiceProxy('/stretch_seeing_eye/detect_door_open', CheckClear)
        self.get_path_service = rospy.ServiceProxy('/move_base/WaypointGlobalPlanner/waypoint_path', MakePath)
        self.client = client.Client('/move_base/DWAPlannerROS', timeout=30, config_callback=None)
        rospy.sleep(1)

        try:
            self.import_data(rospy.get_param('/description_file'))  # type: ignore
        except Exception as e:
            rospy.logerr(e)
            exit(1)

    def check_points(self):
        try:
            transform: tf2_ros.TransformStamped = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        except Exception as e:
            rospy.logerr(e)
            return

        for id, wp in self.waypoints.items():
            d = math.hypot(wp.poseStamped.pose.position.x - transform.transform.translation.x, wp.poseStamped.pose.position.y - transform.transform.translation.y)
            if d < 1:
                for door in wp.doors:
                    message = 'Near ' + self.doors[door].name
                    self.door_msg_pub.publish(DoorMsg(detail_level=self.doors[door].detail_level, data=message, angle=math.atan2(self.doors[door].entrance_pose.pose.position.y - transform.transform.translation.y, self.doors[door].entrance_pose.pose.position.x - transform.transform.translation.x)))
                for w in wp.connections:
                    if self.waypoints[w].navigable:
                        message = 'Near ' + self.waypoints[w].name
                        self.door_msg_pub.publish(DoorMsg(detail_level=self.waypoints[w].detail_level, data=message, angle=math.atan2(self.waypoints[w].poseStamped.pose.position.y - transform.transform.translation.y, self.waypoints[w].poseStamped.pose.position.x - transform.transform.translation.x)))

    def move_base_status_callback(self, msg: GoalStatusArray):
        if len(msg.status_list) and msg.status_list[-1].status == 1:  # type: ignore
            self.moving = True
        else:
            self.moving = False

    def set_max_vel_callback(self, msg: Float32):
        self.max_vel = msg.data
        self.update_move_base('max_vel_x', self.max_vel * self.scaling_factor)

    def handle_callback(self, msg: UInt32):
        low = 0
        high = 1000
        raw_val = (msg.data - low) / (high - low)
        if raw_val < 0.2:
            val = 0
        elif raw_val < 0.4:
            val = raw_val
        elif raw_val < 0.6:
            val = 1
        elif raw_val < 0.8:
            val = 1 + (raw_val - 0.6)
        else:
            val = 2
        if abs(self.scaling_factor - val) > 0.01:
            self.scaling_factor = val
            self.update_move_base('max_vel_x', self.max_vel * self.scaling_factor)

    def stop_navigation_callback(self, msg: TriggerRequest):
        self.move_base_cancel_pub.publish(GoalID())
        self.current_waypoint_pub.publish(UInt32(data=-1))
        self.cancel = True
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
                        # q = quaternion_from_euler(0, 0, math.atan2(d.entrance_pose.pose.position.y - d.inside_pose.pose.position.y, d.entrance_pose.pose.position.x - d.inside_pose.pose.position.x))
                        d.inside_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                        self.goals[d.name + ' Inside'] = d
                        self.goals[d.name + ' Entrance'] = d
                        self.nav_waypoints.append(d.name + ' Inside')
                        self.nav_waypoints.append(d.name + ' Entrance')
                        count += 1
                        markers.append(self.create_marker(
                            d.inside_pose, -d.id, ColorRGBA(1.0, 0.0, 0.0, 1.0)))
                    else:
                        self.goals[d.name + ' Entrance'] = d
                        self.nav_waypoints.append(d.name)
                elif line.startswith('Waypoint'):
                    w = Waypoint(line)
                    self.waypoints[w.id] = w
                    count += 1
                    markers.append(self.create_marker(w.poseStamped, w.id))
                    self.goals[w.name] = w
                    if w.navigable:
                        self.nav_waypoints.append(w.name)
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
        self.adjacency_matrix = {
            'points': points,
            'lookup_table': lookup_table,  # id -> index
            'reverse_lookup_table': {index: id for id, index in lookup_table.items()},  # index -> id
            'adjacency_matrix': [[i for i, v in enumerate(row) if v > 0] for row in adjacency_matrix],
        }
        self.waypoint_pub.publish(msg)

    def wait_for_move_base(self, index=-1):
        self.cancel = False
        while not rospy.is_shutdown() and self.moving:
            self.check_points()
            rospy.sleep(0.1)
        if not self.cancel:
            self.current_waypoint_pub.publish(UInt32(data=index))

    def rotate_routine(self, goal: PoseStamped):
        rospy.logdebug('Rotate routine begin')
        while not rospy.is_shutdown():
            try:
                transform: tf2_ros.TransformStamped = self.tfBuffer.lookup_transform('base_link', 'map', rospy.Time())
                break
            except Exception as e:
                rospy.logerr(e)

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position = transform.transform.translation
        start.pose.orientation = transform.transform.rotation

        req = MakePathRequest()
        req.start = start
        req.goal = goal
        res = self.get_path_service(req)
        if len(res.points) < 2:
            return
        pose1 = self.adjacency_matrix['points'][res.points[0]]
        pose2 = self.adjacency_matrix['points'][res.points[1]]
        q = quaternion_from_euler(0, 0, math.atan2(pose2.pose.position.y - pose1.pose.position.y, pose2.pose.position.x - pose1.pose.position.x))
        pose1.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.update_move_base('yaw_goal_tolerance', 0.05)
        self.move_base_goal_pub.publish(pose1)
        rospy.sleep(2)
        self.wait_for_move_base(index=res.points[0])
        rospy.logdebug('Rotate routine end')

    def entrance_routine(self, goal: str):
        self.rotate_routine(self.goals[goal].entrance_pose)

        self.message_pub.publish(String(data='Navigating to ' + goal))

        self.update_move_base('yaw_goal_tolerance', 0.05)
        self.move_base_goal_pub.publish(self.goals[goal].entrance_pose)
        rospy.sleep(2)
        self.wait_for_move_base(index=self.adjacency_matrix['lookup_table'][self.goals[goal].id])
        return

    def inside_routine(self, goal: str):
        door: Door = self.goals[goal]

        self.rotate_routine(door.inside_pose)

        self.message_pub.publish(String(data='Navigating to ' + goal.replace('Inside', 'Entrance') + ' and then through the door if open'))
        self.update_move_base('yaw_goal_tolerance', 0.05)
        self.move_base_goal_pub.publish(door.entrance_pose)
        rospy.sleep(2)
        self.wait_for_move_base(index=self.adjacency_matrix['lookup_table'][door.id])

        rospy.logdebug('Checking for door')
        req = CheckClearRequest()
        req.p1 = door.entrance_pose
        req.p2 = door.inside_pose
        res = self.check_door_service(req)
        if res.success:
            rospy.logdebug('Door open')
            self.message_pub.publish(String(data='Proceeding through door'))
            self.move_base_goal_pub.publish(door.inside_pose)
            rospy.sleep(2)
            self.wait_for_move_base(index=self.adjacency_matrix['lookup_table'][door.id] + 1)
        else:
            rospy.logdebug('Door closed')
            rospy.logdebug(res.message)
            self.message_pub.publish(String(data='Door closed'))

    def waypoint_routine(self, goal: str):
        waypoint = self.goals[goal]

        self.rotate_routine(waypoint.poseStamped)

        self.message_pub.publish(String(data='Navigating to ' + goal))
        self.update_move_base('yaw_goal_tolerance', 6.28)
        self.move_base_goal_pub.publish(waypoint.poseStamped)
        rospy.sleep(2)
        self.wait_for_move_base(self.adjacency_matrix['lookup_table'][waypoint.id])
        return

    def navigate_to_waypoint(self, msg: WaypointRequest):
        if msg.data is None or msg.data == '':
            return WaypointResponse()

        goal = msg.data
        if goal not in self.goals.keys():
            rospy.logdebug('Goal unknown')
            return WaypointResponse()

        if 'Entrance' in goal:
            self.entrance_routine(goal)
        elif 'Inside' in goal:
            self.inside_routine(goal)
        else:
            self.waypoint_routine(goal)

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
        return GetWaypointsResponse(waypoints=self.nav_waypoints)


if __name__ == '__main__':
    rospy.init_node('navigate_waypoints', log_level=rospy.DEBUG)
    node = NavigateWaypoint()
    rospy.sleep(1)
    # node.navigate_to_waypoint('M1')
    rospy.loginfo('Ready')
    while not rospy.is_shutdown():
        rospy.spin()
