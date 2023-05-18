import rospy

from std_msgs.msg import String
import tf2_ros

from stretch_seeing_eye.srv import Plane, PlaneRequest, PlaneResponse

class DetectFeature:
    def __init__(self):
        self.publish_feature = rospy.Publisher('/stretch_seeing_eye/feature', String, queue_size=1)
        self.create_feature_marker = rospy.ServiceProxy('/stretch_seeing_eye/create_plane_marker', Plane)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.wait_for_service('/stretch_seeing_eye/create_plane_marker')

        self.import_features(rospy.get_param('/features_file'))

    def import_features(self, file: str):
        self.features = {}
        with open(file, 'r') as f:
            data = f.readlines()
            for line in data:
                rospy.logdebug(line.strip())
                line = line.split(',')
                self.features[line[0]] = {}
                self.features[line[0]]['Description'] = line[1].strip()
                self.features[line[0]]['P1'] = {'x': float(line[2]), 'y': float(line[3])}
                self.features[line[0]]['P2'] = {'x': float(line[4]), 'y': float(line[5])}

                self.create_feature_marker(PlaneRequest(x1=self.features[line[0]]['P1']['x'], y1=self.features[line[0]]['P1']['y'], x2=self.features[line[0]]['P2']['x'], y2=self.features[line[0]]['P2']['y']))
    
    def start(self):
        rate = rospy.Rate(10)
        previous_feature = None
        while not rospy.is_shutdown():
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
                # rospy.logdebug(transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # rospy.logdebug('No transform found')
                rate.sleep()
                continue
            
            for feature in self.features:
                if self.is_in_range(transform.transform.translation, self.features[feature]['P1'], self.features[feature]['P2']):
                    if previous_feature != feature:
                        rospy.loginfo('Found feature: {}'.format(feature))
                        self.publish_feature.publish(feature)
                        previous_feature = feature
                    break
            rate.sleep()


    def is_in_range(self, position, p1, p2):
        if position.x > p1['x'] and position.x < p2['x'] or position.x > p2['x'] and position.x < p1['x']:
            if position.y > p1['y'] and position.y < p2['y'] or position.y > p2['y'] and position.y < p1['y']:
                return True
        return False


if __name__ == '__main__':
    rospy.init_node('detect_feature', anonymous=True, log_level=rospy.DEBUG)
    detect_feature = DetectFeature()
    detect_feature.start()