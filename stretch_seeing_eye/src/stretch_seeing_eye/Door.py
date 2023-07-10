import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from stretch_seeing_eye.DetailLevel import DetailLevel

# Door, id, name, description, DetailLevel, x, y, inside?, insideX, insideY


class Door:
    def __init__(self, str: str) -> None:
        dataIter = iter(str.split(','))
        next(dataIter)
        self.id = int(next(dataIter))
        self.name = next(dataIter)
        self.name_lower = self.name.lower()
        self.description = next(dataIter)
        self.detail_level: DetailLevel = getattr(
            DetailLevel, next(dataIter)).value
        self.entrance_pose = PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                Point(float(next(dataIter)), float(next(dataIter)), 0),
                Quaternion(0, 0, 0, 1)
            )
        )
        self.inside = next(dataIter).strip().lower() == 'true'
        if self.inside:
            self.inside_pose = PoseStamped(
                header=Header(frame_id='map'),
                pose=Pose(
                    Point(float(next(dataIter)), float(next(dataIter)), 0),
                    Quaternion(0, 0, 0, 1)
                )
            )
