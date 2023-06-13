import rospy
import tf2_ros
import tf2_geometry_msgs
import math as Math
from geometry_msgs.msg import PointStamped, Point
from stretch_moveit_shim.srv import SetJoints, SetJointsRequest
from stretch_moveit_shim.msg import Joint

class FollowClass():
    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service('/stretch_interface/set_joints')
        self.joint_angle = 0

        self.face_tracking_point_sub = rospy.Subscriber('/stretch_seeing_eye/face_tracking_point', PointStamped, self.callback, queue_size=1)
        self.move_joints = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints)

    def callback(self, point: PointStamped):
        # rospy.logdebug(point)
        angle = Math.atan2(point.point.y, point.point.z) 
        if abs(angle) < 0.1:
            return
        self.joint_angle += angle
        rospy.logdebug(angle)
        req = SetJointsRequest([Joint(joint_name='joint_head_pan', val=self.joint_angle)])
        self.move_joints(req)




if __name__ == '__main__':
    rospy.init_node('face_follow', anonymous=True, log_level=rospy.DEBUG)
    follow = FollowClass()
    rospy.spin()
