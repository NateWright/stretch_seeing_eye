import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from stretch_seeing_eye.DetailLevel import DetailLevel

# name, x, y, connectionCount, connection1, connection2, ..., navigatable, door?, EntranceBool, DetailLevel


class Waypoint:
    def __init__(self, str: str):
        data = str.split(',')
        self.name = data[0]
        self.name_lower = self.name.lower()
        self.poseStamped = PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                Point(float(data[1]), float(data[2]), 0),
                Quaternion(0, 0, 0, 1)
            )
        )
        connectionCount = int(data[3])
        self.connections = []
        for i in range(4, 4 + connectionCount):
            self.connections.append(data[i].strip())

        self.navigatable = data[4 + connectionCount].strip().lower() == 'true'
        self.door = data[5 + connectionCount].strip().lower() == 'true'
        if self.door:
            self.entrance = data[6 +
                                 connectionCount].strip().lower() == 'entrance'
            self.detail_level: DetailLevel = getattr(
                DetailLevel, data[7 + connectionCount]).value
