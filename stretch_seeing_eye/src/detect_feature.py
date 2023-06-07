import rospy
import tf2_ros
import tf2_geometry_msgs
import math as Math

from enum import Enum
from std_msgs.msg import String
from geometry_msgs.msg import Point32, Point, PointStamped
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse

from stretch_seeing_eye.feature import Feature
from stretch_seeing_eye.srv import Feature as FeatureService, FeatureRequest, FeatureResponse

MARKER_TOPIC = '/stretch_seeing_eye/create_marker'

class DetailLevel(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3

class DetectFeature:
    def __init__(self):
        rospy.wait_for_service(MARKER_TOPIC)

        self.set_detail_level_sub = rospy.Subscriber('/stretch_seeing_eye/set_detail_level', String, self.set_detail_level_callback)

        self.publish_feature = rospy.Publisher('/stretch_seeing_eye/feature', String, queue_size=1)

        self.create_feature_marker = rospy.ServiceProxy(MARKER_TOPIC, FeatureService)
        self.make_plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan, persistent=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.parameters = {
            'door_distance': rospy.get_param('detect_feature/Feature_Detection/door_distance'),
            'door_detection_cone': rospy.get_param('detect_feature/Feature_Detection/door_detection_cone'),
            'feature_distance': rospy.get_param('detect_feature/Feature_Detection/feature_distance'),
        }

        self.features = {}
        self.previous_feature = None
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
                f = Feature(line.strip())
                self.features[f.detail_level][f.name] = f
                self.create_feature_marker(FeatureRequest(points=f.points, detail_level=f.detail_level - 1))

    def set_detail_level_callback(self, msg: String):
        self.detail_level = getattr(DetailLevel, msg.data)
        rospy.logdebug('Set detail level to {}'.format(self.detail_level))

    def check_feature_point(self, key: str, feature: Feature):
        try:
            point = PointStamped(point=Point(x=feature.points[0].x, y=feature.points[0].y, z=feature.points[0].z))
            point.header.frame_id = 'map'
            point = self.tf_buffer.transform(point, 'base_link')
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        request = GetPlanRequest()
        request.start.header.frame_id = 'map'
        request.start.pose.position.x = transform.transform.translation.x
        request.start.pose.position.y = transform.transform.translation.y
        request.start.pose.orientation.w = 1.0
        request.goal = feature.getPoseStamped()

        try:
            plan: GetPlanResponse = self.make_plan_service(request)
        except:
            return

        length = 0
        for i, el in enumerate(plan.plan.poses):
            if i == 0:
                continue
            length += Math.sqrt((el.pose.position.x - plan.plan.poses[i-1].pose.position.x)**2 + (el.pose.position.y - plan.plan.poses[i-1].pose.position.y)**2)

        if point.point.x > 0 and length < 5:
            rospy.logdebug('Found feature: {}'.format(key))
            rospy.logdebug('\t Angle: {}'.format(Math.degrees(Math.atan2(point.point.y, point.point.x))))
            self.publish_feature.publish(feature.description)
            self.previous_feature = key
    
    def check_feature_polygon(self, key: str, feature: Feature):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logdebug('No transform found')
            return
        points: ShapelyPoint = []
        for point in feature.points:
            points.append(ShapelyPoint(point.x, point.y))
        polygon = ShapelyPolygon(points)
        if polygon.distance(ShapelyPoint(transform.transform.translation.x, transform.transform.translation.y)) <= self.parameters['feature_distance']:
            rospy.logdebug('Found feature: {}'.format(feature.name))
            self.publish_feature.publish(feature.description)
            self.previous_feature = key

    def check_range(self):
        for detail_level in DetailLevel:
            if detail_level.value > self.detail_level.value:
                break
            for key, value in self.features[detail_level.value].items():
                assert isinstance(value, Feature)
                if len(value.points) == 1 and self.previous_feature != key:
                    self.check_feature_point(key, value)
                elif len(value.points) == 4 and self.previous_feature != key:
                    self.check_feature_polygon(key, value)

    
    def start(self):
        rate = rospy.Rate(10)
        self.previous_feature = None
        while not rospy.is_shutdown():
            self.check_range()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('detect_feature', anonymous=True, log_level=rospy.DEBUG)
    detect_feature = DetectFeature()
    detect_feature.start()