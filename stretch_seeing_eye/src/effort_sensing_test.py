import rospy
import math as Math

from sensor_msgs.msg import JointState
from stretch_moveit_shim.srv import SetJoints, SetJointsRequest, SetJointsResponse
from stretch_moveit_shim.msg import Joint

#   - joint_arm
#   - joint_lift
#   - joint_head_pan
#   - joint_wrist_pitch
#   - joint_wrist_roll
#   - joint_wrist_yaw
#   - joint_gripper_finger_left
#   - joint_head_tilt
#   - joint_arm_l3
#   - joint_arm_l2
#   - joint_arm_l1
#   - joint_arm_l0
#   - wrist_extension
#   - gripper_aperture
#   - joint_gripper_finger_right

class JointEffort():
    def __init__(self):
        self.joints = None
        self.last_value = 0.0
        self.last_message = ""
        self.moving_position = 4.5
        self.door_position = 4.0
    
        rospy.wait_for_service('/stretch_interface/set_joints')
        self.service = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints)
        self.service([Joint(joint_name='joint_wrist_yaw', val=self.door_position)])
    
        self.sub = rospy.Subscriber("/stretch/joint_states", JointState, self.callback)


    def callback(self, msg: JointState):
        if self.joints is None:
            self.joints = {}
            for i, val in enumerate(msg.name):
                self.joints[val] = i
            rospy.logdebug(self.joints)

        curr_val = msg.effort[self.joints['joint_wrist_yaw']]

        if abs(self.last_value - curr_val) < 0.2 and self.last_message != (msg := 'Doing Nothing'):
            pass
        elif self.last_value - curr_val < 0 and self.last_message != (msg := 'Speeding Up'):
            self.last_message = msg
            rospy.logdebug(self.last_message)
        elif self.last_value - curr_val > 0 and self.last_message != (msg := 'Slowing Down'):
            self.last_message = msg
            rospy.logdebug(self.last_message)
        self.last_value = curr_val
        


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True, log_level=rospy.DEBUG)
    joint_effort = JointEffort()
    rospy.spin()