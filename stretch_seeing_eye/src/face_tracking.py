# Reference: https://github.com/elpimous/ros-face-recognition
import rospy
import cv2
import dlib
import ros_numpy

import math as Math
import numpy as np

from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped

class TestClass():
    def __init__(self):
        self.detector = dlib.get_frontal_face_detector()
        self.old_faces = []
        self.pointcloud_sub =rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback, queue_size=1)
        self.face_tracking_pub = rospy.Publisher('/stretch_seeing_eye/face_tracking', Image, queue_size=10)
        self.face_tracking_point_pub = rospy.Publisher('/stretch_seeing_eye/face_tracking_point', PointStamped, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, cloud: PointCloud2):
        pc2_arr = ros_numpy.point_cloud2.pointcloud2_to_array(cloud)
        arr = ros_numpy.point_cloud2.split_rgb_field(pc2_arr)
        pointcloud_to_image = lambda x: np.uint8((np.uint(x['r']) + np.uint(x['g']) + np.uint(x['b'])) / 3)
        image = pointcloud_to_image(arr)
        # image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

        faces = self.detector(image, 1)
        if len(self.old_faces) < len(faces):
            self.old_faces = []
            for face in faces:
                tracker = dlib.correlation_tracker()
                tracker.start_track(image, face)
                self.old_faces.append(tracker)
        else:
            for i, tracker in enumerate(self.old_faces):
                quality = tracker.update(image)
                if quality > 7:
                    pos = tracker.get_position()
                    pos = dlib.rectangle(
                        int(pos.left()),
                        int(pos.top()),
                        int(pos.right()),
                        int(pos.bottom()),
                    )
                    center = (int(pos.left() + pos.width() / 2), int(pos.top() + pos.height() / 2))
                    p = PointStamped()
                    p.header = cloud.header
                    p.header.frame_id = cloud.header.frame_id
                    x = center[1] * 2
                    y = pc2_arr.shape[0] - 1 - center[0] * 2
                    p.point.x = pc2_arr[y][x][0]
                    p.point.y = pc2_arr[y][x][1]
                    p.point.z = pc2_arr[y][x][2]
                    if x >= 0 and x < pc2_arr.shape[1] and y >= 0 and y < pc2_arr.shape[0]:
                        self.face_tracking_point_pub.publish(p)
                        
                    cv2.circle(image, center, radius=4, color=(0, 0, 255), thickness=-1)
                    cv2.rectangle(image, (pos.left(), pos.top()), (pos.right(), pos.bottom()),
                                (100, 200, 100))
                else:
                    self.old_faces.pop(i)
        # publish image
        msg = self.bridge.cv2_to_imgmsg(image, '8UC1')
        msg.header = cloud.header
        self.face_tracking_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('face_tracking', anonymous=True, log_level=rospy.DEBUG)
    node = TestClass()
    rospy.spin()
