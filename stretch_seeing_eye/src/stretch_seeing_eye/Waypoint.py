import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from stretch_seeing_eye.DetailLevel import DetailLevel

# Waypoint, id, name, x, y, connectionCount, ..., doorCount, ..., navigatable, detailLevel?


class Waypoint:
    def __init__(self, str: str):
        dataIter = iter(str.split(','))
        next(dataIter)  # Skip Waypoint
        self.id = int(next(dataIter))
        self.name = next(dataIter)
        self.name_lower = self.name.lower()

        self.poseStamped = PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                Point(float(next(dataIter)), float(next(dataIter)), 0),
                Quaternion(0, 0, 0, 1)
            )
        )
        connectionCount = int(next(dataIter))
        self.connections: list[int] = []
        for _ in range(connectionCount):
            self.connections.append(int(next(dataIter)))
        doorCount = int(next(dataIter))
        self.doors: list[int] = []
        for _ in range(doorCount):
            self.doors.append(int(next(dataIter)))
        self.navigatable = next(dataIter).strip().lower() == 'true'
        if self.navigatable:
            self.detail_level: DetailLevel = getattr(
                DetailLevel, next(dataIter)).value

    def getPoseStamped(self) -> PoseStamped:
        return self.poseStamped
