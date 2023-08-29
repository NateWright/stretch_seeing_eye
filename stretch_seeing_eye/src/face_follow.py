import rospy
import tf2_ros
import tf2_geometry_msgs
import math as Math
from geometry_msgs.msg import PointStamped, Point
from stretch_moveit_shim.srv import SetJoints, SetJointsRequest
from stretch_moveit_shim.msg import Joint, Joints
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement


class FollowClass():
    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service('/stretch_interface/set_joints')
        self.joint_angle = 0

        self.face_tracking_point_sub = rospy.Subscriber('/face_detector/people_tracker_measurements_array', PositionMeasurementArray, self.callback, queue_size=1)
        self.move_joints = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints)
        self.move_joints_pub = rospy.Publisher('/stretch_interface/set_joints', Joints, queue_size=10)
        self.debug_pub = rospy.Publisher('/stretch_seeing_eye/face_follow/debug_point', PointStamped, queue_size=10)
        self.point_stamped = PointStamped()
        self.point_stamped.header.frame_id = 'camera_color_optical_frame'

    def callback(self, arr: PositionMeasurementArray):
        # rospy.logdebug("got message")
        if len(arr.people) < 1:
            return
        self.point_stamped.point = arr.people[0].pos
        self.debug_pub.publish(self.point_stamped)
        p = self.tf_buffer.transform(self.point_stamped, 'base_link')
        angle = Math.atan2(p.point.y, p.point.x)
        if abs(angle) < 0.1:
            return
        # self.joint_angle += angle
        j = Joints()
        j.joints = [Joint(joint_name='joint_head_pan', val=angle)]
        self.move_joints_pub.publish(j)
        # rospy.logdebug(angle)
        # req = SetJointsRequest([Joint(joint_name='joint_head_pan', val=angle)])
        # self.move_joints(req)




if __name__ == '__main__':
    rospy.init_node('face_follow', anonymous=True, log_level=rospy.DEBUG)
    follow = FollowClass()
    rospy.spin()
