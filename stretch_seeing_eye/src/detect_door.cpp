#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>  // for pcl::removeNaNFromPointCloud
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <iterator>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

using Eigen::Vector4f;

float distancePointToPlane(const Point &point, const Vector4f &plane_parameters) {
    return std::abs(plane_parameters[0] * point.x + plane_parameters[1] * point.y +
                    plane_parameters[2] * point.z + plane_parameters[3]) /
           std::sqrt(plane_parameters[0] * plane_parameters[0] + plane_parameters[1] * plane_parameters[1] +
                     plane_parameters[2] * plane_parameters[2]);
}

float distancePointToLine(const Point &point, const Point &line_p1, const Point &line_p2) {
    return std::abs(
               (line_p2.x - line_p1.x) * (line_p1.y - point.y) - (line_p1.x - point.x) * (line_p2.y - line_p1.y)) /
           std::sqrt(
               (line_p2.x - line_p1.x) * (line_p2.x - line_p1.x) + (line_p2.y - line_p1.y) * (line_p2.y - line_p1.y));
}

Point averageCloud(const PointCloud::Ptr &cloud) {
    Point sum;
    sum.x = 0;
    sum.y = 0;
    sum.z = 0;
    for (auto &point : *cloud) {
        sum.x += point.x;
        sum.y += point.y;
        sum.z += point.z;
    }
    sum.x /= cloud->size();
    sum.y /= cloud->size();
    sum.z /= cloud->size();
    return sum;
}

class DoorDetector {
   public:
    DoorDetector(ros::NodeHandlePtr nh) : nh(nh), tfListener(tfBuffer) {
        // cloudSub = nh->subscribe("/test_pointcloud", 1, &DoorDetector::door_detection, this);
        // imageSub = nh->subscribe("/test_image", 1, &DoorDetector::door_detection_image, this);

        service = nh->advertiseService("/stretch_seeing_eye/detect_door_open", &DoorDetector::door_detection_srv, this);

        testCloudPub = nh->advertise<PointCloud>("/stretch_seeing_eye/door_detector/test_pcd", 1);
        testHullPub = nh->advertise<PointCloud>("/stretch_seeing_eye/door_detector/test_hull", 1);

        testImagePub = nh->advertise<sensor_msgs::Image>("/test_img", 1);
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/rviz_visual_markers"));
    }
    bool door_detection_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        const boost::shared_ptr<const PointCloud> pc = ros::topic::waitForMessage<PointCloud>("/camera/depth/color/points");
        if (!pc) {
            ROS_ERROR_STREAM("No pointcloud received");
            res.success = false;
            return true;
        }
        door_detection(pc);
        res.success = door;
        res.message = message;
        return true;
    }

    // void door_detection_image(const sensor_msgs::ImagePtr msg) {
    //     cv_bridge::CvImagePtr cv_ptr;
    //     try {
    //         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    //     } catch (cv_bridge::Exception &e) {
    //         ROS_ERROR("cv_bridge exception: %s", e.what());
    //         return;
    //     }
    //     cv::Mat detected_edges;
    //     cv::blur(cv_ptr->image, detected_edges, cv::Size(3, 3));
    //     if (!detected_edges.isContinuous()) {
    //         ROS_ERROR("Image is not continuous");
    //         return;
    //     }
    //     std::vector<uchar> detected_edges_vec(detected_edges.size().area() / 2 + 1);
    //     std::partial_sort_copy(detected_edges.begin<uchar>(), detected_edges.end<uchar>(),
    //                            detected_edges_vec.begin(), detected_edges_vec.end());
    //     int md = detected_edges_vec.back();
    //     int lower_value = std::max(0, (int)((1.0 - 0.33) * (float)md));
    //     int upper_value = std::min(255, (int)((1.0 + 0.33) * (float)md));
    //     cv::Canny(detected_edges, detected_edges, lower_value, upper_value);

    //     cv_ptr->image = detected_edges;
    //     testImagePub.publish(cv_ptr->toImageMsg());
    // }

    void door_detection(const PointCloud::ConstPtr &cloud) {
        PointCloud::Ptr cloudTransformed(new PointCloud);
        const geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base_link", cloud->header.frame_id, ros::Time(0));
        pcl_ros::transformPointCloud(*cloud, *cloudTransformed, transformStamped.transform);
        cloudTransformed->header.frame_id = "base_link";
        PointCloud::Ptr voxel_cloud = voxelize_cloud(cloudTransformed);
        PointCloud::Ptr distance_cloud = filterDistance(voxel_cloud);
        findWall(distance_cloud);
        // Point p = averageCloud(wall_cloud);
        // ROS_INFO_STREAM(p);
        // float distance = distancePointToPlane(p, plane_parameters);
        // ROS_INFO_STREAM(distance);

        // testCloudPub.publish(wall_cloud);
    }

    PointCloud::Ptr voxelize_cloud(const PointCloud::ConstPtr &cloud) {
        PointCloud::Ptr voxel_cloud(new PointCloud);
        pcl::VoxelGrid<Point> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel_filter.filter(*voxel_cloud);
        return voxel_cloud;
    }
    PointCloud::Ptr filterDistance(const PointCloud::ConstPtr &cloud) {
        PointCloud::Ptr segmented_cloud(new PointCloud);
        pcl::IndicesPtr indices(new std::vector<int>);
        pcl::PassThrough<Point> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.2, 3);
        pass.filter(*indices);

        pcl::ExtractIndices<Point> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.filter(*segmented_cloud);
        return segmented_cloud;
    }
    void findWall(const PointCloud::ConstPtr &cloud) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // Set of points in the plane
        pcl::SACSegmentation<Point> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // pcl::ExtractIndices<Point> extract;
        // extract.setInputCloud(cloud);
        // extract.setIndices(inliers);
        // extract.filter(*wall_cloud);
        // return wall_cloud;

        plane_parameters = {coefficients->values[0], coefficients->values[1],
                            coefficients->values[2], coefficients->values[3]};

        PointCloud::Ptr wall_cloud(new PointCloud);
        pcl::ProjectInliers<Point> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setIndices(inliers);
        proj.setModelCoefficients(coefficients);
        proj.filter(*wall_cloud);

        PointCloud::Ptr wall_hull(new PointCloud);
        std::vector<pcl::Vertices> polygons;
        pcl::ConcaveHull<Point> hull;
        hull.setInputCloud(wall_cloud);
        hull.setAlpha(0.04);
        hull.reconstruct(*wall_hull, polygons);

        if (polygons.size() == 0) {
            ROS_ERROR_STREAM("No polygons found");
            message = "No polygons found";
            door = false;
            return;
        }
        pcl::Vertices v = polygons[0];
        std::vector<std::vector<Point>> lines;
        lines.push_back({wall_hull->points[v.vertices[0]]});
        ROS_INFO_STREAM(v.vertices.size());
        typedef enum { LEFT,
                       RIGHT,
                       UP,
                       DOWN } Direction;
        std::vector<Direction> directions;
        Direction last_direction = UP, d;

        auto f = [](Point p1, Point p2) {
            auto y = p2.y - p1.y;
            auto z = p2.z - p1.z;
            if (std::abs(y) > std::abs(z)) {
                if (y > 0) {
                    return RIGHT;
                } else {
                    return LEFT;
                }
            } else {
                if (z > 0) {
                    return UP;
                } else {
                    return DOWN;
                }
            }
        };
        last_direction = f(wall_hull->points[v.vertices[0]], wall_hull->points[v.vertices[1]]);
        directions.push_back(last_direction);

        for (size_t i = 2; i < v.vertices.size(); i++) {
            d = f(wall_hull->points[v.vertices[i - 1]], wall_hull->points[v.vertices[i]]);
            if (d != last_direction) {
                lines.push_back({wall_hull->points[v.vertices[i - 1]]});
                last_direction = d;
                directions.push_back(last_direction);
            } else {
                lines.back().push_back(wall_hull->points[v.vertices[i - 1]]);
            }
        }

        auto dist = [](geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
            return (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z);
        };

        visual_tools_->deleteAllMarkers();
        // for (auto &line : lines) {
        //     geometry_msgs::Point p1, p2;
        //     p1.x = line[0].x;
        //     p1.y = line[0].y;
        //     p1.z = line[0].z;
        //     p2.x = line.back().x;
        //     p2.y = line.back().y;
        //     p2.z = line.back().z;
        //     if ((p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z) > 0.5 * 0.5)
        //         visual_tools_->publishLine(p1, p2, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
        // }
        ROS_INFO_STREAM("line count: " << lines.size());
        if (directions.size() != lines.size()) {
            ROS_ERROR_STREAM("directions and lines size mismatch");
            message = "directions and lines size mismatch";
            door = false;
            return;
        }
        for (size_t i = 0; i < directions.size() - 2; i++) {
            if (directions[i] == Direction::UP && (directions[i + 1] == Direction::RIGHT || directions[i + 1] == Direction::LEFT) &&
                directions[i + 2] == Direction::DOWN) {
                ROS_INFO_STREAM("Found door");
                message = "Found door";
                door = true;
                return;
                for (size_t j = i; j < i + 3; j++) {
                    geometry_msgs::Point p1, p2;
                    p1.x = lines[j][0].x;
                    p1.y = lines[j][0].y;
                    p1.z = lines[j][0].z;
                    p2.x = lines[j].back().x;
                    p2.y = lines[j].back().y;
                    p2.z = lines[j].back().z;
                    visual_tools_->publishLine(p1, p2, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
                }
                break;
            }
        }
        visual_tools_->trigger();
        testHullPub.publish(wall_hull);

        // Picks out points behind plane
        // pcl::PlaneClipper3D<Point> clipper(plane_parameters);
        // pcl::Indices filterIndicies;  // Set of points behind the plane including the plane
        // clipper.clipPointCloud3D(*cloud, filterIndicies);

        // pcl::Indices indices;
        // std::set_difference(filterIndicies.begin(), filterIndicies.end(), inliers->indices.begin(),
        //                     inliers->indices.end(), std::inserter(indices, indices.begin()));

        // PointCloud::Ptr wall_cloud(new PointCloud);
        // pcl::copyPointCloud(*cloud, indices, *wall_cloud);

        // return wall_cloud;
        testCloudPub.publish(wall_cloud);
        message = "No door found from hull";
        door = false;
        return;
    }

   private:
    ros::NodeHandlePtr nh;

    ros::ServiceServer service;

    ros::Subscriber cloudSub;
    ros::Subscriber imageSub;

    ros::Publisher testCloudPub;
    ros::Publisher testHullPub;
    ros::Publisher testImagePub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    bool door;
    std::string message;
    Vector4f plane_parameters;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_door");
    ros::NodeHandlePtr nh(new ros::NodeHandle);
    DoorDetector doorDetector(nh);
    ros::spin();
    return 0;
}
