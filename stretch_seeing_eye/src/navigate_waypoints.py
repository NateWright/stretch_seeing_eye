import rospy
import math

from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalID
from dynamic_reconfigure import client
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

from stretch_seeing_eye.shortest_path import Graph
from stretch_seeing_eye.srv import Waypoint, WaypointRequest, WaypointResponse

class NavigateWaypoint:
    def __init__(self):
        # Publishers and Subscribers
        self.move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.waypoint_rviz_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        self.move_base_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback, queue_size=1)
        # self.command_sub = rospy.Subscriber('/stretch_seeing_eye/command', String, self.command_callback, queue_size=1)
        self.set_curr_waypoint_sub = rospy.Subscriber('/stretch_seeing_eye/set_curr_waypoint', String, self.set_curr_waypoint_callback, queue_size=1)

        self.navigate_to_waypoint_service = rospy.Service('/stretch_seeing_eye/navigate_to_waypoint', Waypoint, self.navigate_to_waypoint)
        self.pause_navigation_service = rospy.Service('/stretch_seeing_eye/pause_navigation', SetBool, self.pause_navigation_callback)

        # self.client = client.Client('/move_base/DWAPlannerROS', timeout=30, config_callback=None)
        rospy.sleep(1)

        self.moving = False
        self.pause = False
        self.waypoints = {}
        self.lookup_table = {}
        self.lookup_table_reverse = {}
        self.curr_waypoint = 'base'
        self.curr_goal = None
        self.import_data(rospy.get_param('/waypoints_file'))

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

    
    def command_callback(self, msg: String):
        self.navigate_to_waypoint(msg.data)
    
    def set_curr_waypoint_callback(self, msg: String):
        self.curr_waypoint = msg.data
    
    def import_data(self, file: str):
        connections = {}
        count = 0

        markers = MarkerArray(markers=[])
        with open(file, 'r') as f:
            data = f.readlines()
            for line in data:
                line = line.split(',')
                name = line[0].strip().lower()
                self.waypoints[name] = PoseStamped(header=Header(frame_id='map'), pose=Pose(Point(float(line[1]), float(line[2]), float(line[3])), Quaternion(float(line[4]), float(line[5]), float(line[6]), float(line[7]))))
                markers.markers.append(self.create_marker(self.waypoints[name].pose.position, self.waypoints[name].pose.orientation, count))
                waypoint_connections = []
                for i in range(8, len(line)):
                    waypoint_connections.append(line[i].strip().lower())
                connections[name] = waypoint_connections
                self.lookup_table[name] = count
                self.lookup_table_reverse[count] = name
                count += 1
        self.waypoint_rviz_pub.publish(markers)
        rows, cols = count, count
        adjacency_matrix = [[0 for i in range(cols)] for j in range(rows)]
        for key, val in connections.items():
            curr_waypoint = self.waypoints[key].pose.position
            for connection in val:
                connection_waypoint = self.waypoints[connection].pose.position
                adjacency_matrix[self.lookup_table[key]][self.lookup_table[connection]] = math.sqrt((curr_waypoint.x - connection_waypoint.x)**2 + (curr_waypoint.y - connection_waypoint.y)**2)
        self.graph = Graph(count)
        self.graph.graph = adjacency_matrix

    def navigate(self, waypoint: str = 'Base'):
        rospy.logdebug('Navigating to ' + waypoint)
        rospy.logdebug(self.waypoints[waypoint])

        self.curr_goal = waypoint
        self.move_pub.publish(self.waypoints[waypoint])
        rospy.logdebug('Published')

    def navigate_to_waypoint(self, msg: WaypointRequest):
        if msg.data not in self.waypoints.keys():
            rospy.logdebug('Waypoint unknown')
            return WaypointResponse()
        waypoints = list(map(lambda x: self.lookup_table_reverse[x], self.graph.dijkstra(self.lookup_table[self.curr_waypoint], self.lookup_table[msg.data])))
        waypoints.append(msg.data)
        waypoints = waypoints[1:]
        rospy.logdebug(waypoints)
        # self.set_navigation_tolerance(2 * math.pi)
        for i, waypoint in enumerate(waypoints):
            # if i == len(waypoints) - 1:
            #     self.set_navigation_tolerance(0.1)
            self.navigate(waypoint)
            self.curr_waypoint = waypoint
            rospy.sleep(1)
            while self.moving or self.pause:
                rospy.sleep(1)
        self.curr_waypoint = msg.data
        rospy.logdebug('Reached ' + msg.data)
        return WaypointResponse()
    
    # def set_navigation_tolerance(self, tolerance: float = 0.1):
    #     self.client.update_configuration({'yaw_goal_tolerance': tolerance})
    
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




if __name__ == '__main__':
    rospy.init_node('navigate_waypoints', log_level=rospy.DEBUG)
    node = NavigateWaypoint()
    rospy.sleep(1)
    # node.navigate_to_waypoint('M1')
    while not rospy.is_shutdown():
        rospy.spin()