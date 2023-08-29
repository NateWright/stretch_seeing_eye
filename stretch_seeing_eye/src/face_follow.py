import rospy
import tf2_ros
import tf2_geometry_msgs
import math as Math
from geometry_msgs.msg import PointStamped, Point
from stretch_moveit_shim.srv import SetJoints, SetJointsRequest
from stretch_moveit_shim.msg import Joint
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement


class FollowClass():
    def __init__(self) -> None:
        # self.tf_buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service('/stretch_interface/set_joints')
        self.joint_angle = 0

        self.face_tracking_point_sub = rospy.Subscriber('/face_detector/people_tracker_measurements_array', PositionMeasurementArray, self.callback, queue_size=1)
        self.move_joints = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints)

    def callback(self, arr: PositionMeasurementArray):
        # rospy.logdebug("got message")
        if len(arr.people) < 1:
            return
        angle = Math.atan2(arr.people[0].pos.x, arr.people[0].pos.y)
        rospy.logdebug(arr.people[0].pos)
        if abs(angle) < 0.05:
            return
        # self.joint_angle += angle
        rospy.logdebug(angle)
        req = SetJointsRequest([Joint(joint_name='joint_head_pan', val=angle)])
        self.move_joints(req)




if __name__ == '__main__':
    rospy.init_node('face_follow', anonymous=True, log_level=rospy.DEBUG)
    follow = FollowClass()
    rospy.spin()
