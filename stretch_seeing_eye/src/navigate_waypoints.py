import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray
from dynamic_reconfigure import client

class NavigateWaypoint:
    def __init__(self):
        # Publishers and Subscribers
        self.move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback, queue_size=1)
        self.client = client.Client('/move_base/TrajectoryPlannerROS', timeout=30, config_callback=None)

        self.moving = False
        self.waypoints = {}
        self.curr_waypoint = 'Base'
        self.import_data(rospy.get_param('/waypoints_file'))

    def move_base_status_callback(self, msg: GoalStatusArray):
        if len(msg.status_list) and msg.status_list[-1].status == 1:
            self.moving = True
        else:
            self.moving = False
        # rospy.logdebug('Moving: ' + str(self.moving))
    
    def import_data(self, file: str):
        with open(file, 'r') as f:
            data = f.readlines()
            for line in data:
                line = line.split(',')
                self.waypoints[line[0]] = PoseStamped(header=Header(frame_id='map'), pose=Pose(Point(float(line[1]), float(line[2]), float(line[3])), Quaternion(float(line[4]), float(line[5]), float(line[6]), float(line[7]))))
                # rospy.loginfo(self.waypoints[line[0]])

    def navigate(self, waypoint: str = 'Base'):
        rospy.logdebug('Navigating to ' + waypoint)
        rospy.logdebug(self.waypoints[waypoint])

        self.move_pub.publish(self.waypoints[waypoint])
        rospy.logdebug('Published')

    def navigate_waypoints(self, waypoints: list = []):
        for waypoint in waypoints:
            self.navigate(waypoint)
            self.curr_waypoint = waypoint
            rospy.sleep(1)
            while self.moving:
                rospy.sleep(1)




if __name__ == '__main__':
    rospy.init_node('navigate_waypoints', log_level=rospy.DEBUG)
    node = NavigateWaypoint()
    rospy.sleep(1)
    node.navigate_waypoints(['R1', 'R2', 'M1'])
    while not rospy.is_shutdown():
        rospy.spin()