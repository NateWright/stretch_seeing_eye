import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
class Waypoint:
    def __init__(self, str: str):
        str = str.split(',')
        self.name = str[0].lower()
        self.poseStamped = PoseStamped(header=Header(frame_id='map'), pose=Pose(Point(float(str[1]), float(str[2]), float(str[3])), Quaternion(float(str[4]), float(str[5]), float(str[6]), float(str[7]))))
        self.connections = []
        for i in range(8, len(str)):
            self.connections.append(str[i].strip().lower())