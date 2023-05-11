import rospy

from geometry_msgs.msg import PoseStamped

def callback(msg: PoseStamped):
    print(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, sep=', ')

if __name__ == '__main__':
    rospy.init_node('createpoints')
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
    rospy.spin()