import rospy
import tf2_ros
import tf2_geometry_msgs
import math as Math
from geometry_msgs.msg import PointStamped, Point
from stretch_moveit_shim.msg import Joint
from stretch_moveit_shim.srv import SetJoints, SetJointsRequest
from stretch_seeing_eye.srv import FindFace, FindFaceRequest, FindFaceResponse
from people_msgs.msg import PositionMeasurementArray, PositionMeasurement


class FaceFinder():
    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service('/stretch_interface/set_joints')

        self.find_face_srv = rospy.Service('/stretch_seeing_eye/find_face', FindFace, self.service_callback)
        self.move_joints = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints)

        self.found = False
        self.point_stamped = PointStamped()

    def service_callback(self, req: FindFaceRequest):
        self.found = False

        face_tracking_point_sub = rospy.Subscriber('/face_detector/people_tracker_measurements_array', PositionMeasurementArray, self.callback, queue_size=1)

        req = SetJointsRequest([Joint(joint_name='joint_head_tilt', val=angle)])
        self.move_joints(req)
        
        angle = -4.0
        while not rospy.is_shutdown() and not self.found:
            req = SetJointsRequest([Joint(joint_name='joint_head_pan', val=angle)])
            self.move_joints(req)
            rospy.sleep(1)
            angle += 0.1
            if self.found or angle > 1.8:
                break

        face_tracking_point_sub.unregister()

        res = FindFaceResponse()
        res.found = self.found
        res.point = self.point_stamped
        return res


    def callback(self, arr: PositionMeasurementArray):
        for i, p in enumerate(arr.people):
            if p.reliability < 0.8:
                rospy.logdebug('reliability too low')
                continue
            self.found = True
            ps = PointStamped()
            ps.header.frame_id = arr.header.frame_id
            ps.point = p.pos
            self.point_stamped = self.tf_buffer.transform(ps, 'base_link')




if __name__ == '__main__':
    rospy.init_node('find_face', anonymous=True, log_level=rospy.DEBUG)
    follow = FaceFinder()
    rospy.spin()
