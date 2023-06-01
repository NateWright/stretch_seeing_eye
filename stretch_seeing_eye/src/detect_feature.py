import rospy
import tf2_ros
import tf2_geometry_msgs
import math as Math

from enum import Enum
from std_msgs.msg import String
from geometry_msgs.msg import Point32, Point, PointStamped
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon

from stretch_seeing_eye.srv import Feature, FeatureRequest, FeatureResponse

MARKER_TOPIC = '/stretch_seeing_eye/create_marker'

class DetailLevel(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3

class DetectFeature:
    def __init__(self):
        rospy.wait_for_service(MARKER_TOPIC)

        self.publish_feature = rospy.Publisher('/stretch_seeing_eye/feature', String, queue_size=1)
        self.create_feature_marker = rospy.ServiceProxy(MARKER_TOPIC, Feature)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.parameters = {
            'door_distance': rospy.get_param('detect_feature/Feature_Detection/door_distance'),
            'door_detection_cone': rospy.get_param('detect_feature/Feature_Detection/door_detection_cone'),
            'feature_distance': rospy.get_param('detect_feature/Feature_Detection/feature_distance'),
        }

        rospy.logdebug(self.parameters)

        self.features = {}
        self.detail_level: DetailLevel = getattr(DetailLevel, rospy.get_param('detect_feature/Feature_Detection/detail_level'))
        
        self.import_features(rospy.get_param('/description_file'))

    def import_features(self, file: str):
        self.features = {}
        for key in DetailLevel:
            self.features[key.value] = {}

        with open(file, 'r') as f:
            data = f.read().split('---\n')[1].split('\n')
            for line in data:
                if line == "":
                    continue
                rospy.logdebug(line.strip())
                line = line.strip().split(',')

                name = line[0]
                description = line[1]
                count = line[2]
                points: Point32 = []
                index = 3
                for i in range(int(count)):
                    points.append(Point32(x=float(line[index]), y=float(line[index+1]), z=0.0))
                    index += 2
                
                detail_level: DetailLevel = getattr(DetailLevel, line[index]).value

                self.features[detail_level][name] = {'Description': description, 'Points': points}

                self.create_feature_marker(FeatureRequest(points=points, detail_level=detail_level - 1))
    
    def start(self):
        rate = rospy.Rate(10)
        previous_feature = None
        while not rospy.is_shutdown():
            for detail_level in DetailLevel:
                if detail_level.value > self.detail_level.value:
                    break
                for key, value in self.features[detail_level.value].items():
                    if len(value['Points']) == 1 and previous_feature != key:
                        try:
                            point = PointStamped()
                            point.point.x = value['Points'][0].x
                            point.point.y = value['Points'][0].y
                            point.point.z = value['Points'][0].z
                            point.header.frame_id = 'map'
                            point = self.tf_buffer.transform(point, 'base_link')
                            transform = self.tf_buffer.lookup_transform('base_link', 'map', rospy.Time())
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            continue
                        if (point.point.x)**2 + (point.point.y)**2 < (self.parameters['door_distance'])**2 and abs(Math.atan(point.point.y/point.point.x)) < self.parameters['door_detection_cone']:
                            rospy.logdebug('Found feature: {}'.format(key))
                            self.publish_feature.publish(value['Description'])
                            previous_feature = key
                            break
                    elif len(value['Points']) == 4 and previous_feature != key:
                        try:
                            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
                            # rospy.logdebug(transform)
                            points: ShapelyPoint = []
                            for point in value['Points']:
                                points.append(ShapelyPoint(point.x, point.y))
                            polygon = ShapelyPolygon(points)
                            if polygon.distance(ShapelyPoint(transform.transform.translation.x, transform.transform.translation.y)) <= self.parameters['feature_distance']:
                                rospy.logdebug('Found feature: {}'.format(value))
                                self.publish_feature.publish(value['Description'])
                                previous_feature = key
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            rospy.logdebug('No transform found')
                            continue
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('detect_feature', anonymous=True, log_level=rospy.DEBUG)
    detect_feature = DetectFeature()
    detect_feature.start()