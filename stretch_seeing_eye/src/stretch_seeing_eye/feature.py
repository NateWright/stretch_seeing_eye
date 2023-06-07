import rospy
from enum import Enum
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, PoseStamped

class DetailLevel(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3

class Feature:
    def __init__(self, str: str):

        data = str.strip().split(',')

        self.name = data[0]
        self.description = data[1]
        count = data[2]
        self.points: Point32 = []
        index = 3
        for i in range(int(count)):
            self.points.append(Point32(x=float(data[index]), y=float(data[index+1]), z=0.0))
            index += 2
        
        self.detail_level: DetailLevel = getattr(DetailLevel, data[index]).value

        self.waypoint = None
        if len(data) > index + 1:
            self.waypoint = data[index+1].lower()
    
    def getPoseStamped(self):
        p = PoseStamped()
        p.pose.position.x = self.points[0].x
        p.pose.position.y = self.points[0].y
        p.pose.position.z = self.points[0].z
        p.pose.orientation.w = 1.0
        p.header = Header(frame_id='map')
        return p

