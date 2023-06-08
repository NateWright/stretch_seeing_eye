# Reference: https://github.com/elpimous/ros-face-recognition
import rospy
import cv2
import dlib
import message_filters

from cv_bridge import CvBridge

from std_msgs.msg import Float32
from sensor_msgs.msg import Image

class TestClass():
    def __init__(self):
        self.detector = dlib.get_frontal_face_detector()
        self.old_faces = []
        # self.test = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        cloud_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.time_sync = message_filters.ApproximateTimeSynchronizer([image_sub, cloud_sub], 1, 0.1)
        self.time_sync.registerCallback(self.callback)
        self.face_tracking_pub = rospy.Publisher('/stretch_seeing_eye/face_tracking', Image, queue_size=10)
        self.face_depth_pub = rospy.Publisher('/stretch_seeing_eye/face_tracking_depth', Float32, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, data: Image, data_depth: Image):
    # def callback(self, data: Image):
        rospy.logdebug('callback')
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(data_depth, 'passthrough')
        image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)

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
                    self.face_depth_pub.publish(depth_image[center[1], center[0]] * 0.001)
                    cv2.circle(image, center, radius=4, color=(0, 0, 255), thickness=-1)
                    cv2.rectangle(image, (pos.left(), pos.top()), (pos.right(), pos.bottom()),
                                (100, 200, 100))
                else:
                    self.old_faces.pop(i)
        # publish image
        self.face_tracking_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))


if __name__ == '__main__':
    rospy.init_node('face_tracking', anonymous=True, log_level=rospy.DEBUG)
    node = TestClass()
    rospy.spin()
