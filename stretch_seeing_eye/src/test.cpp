#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "stretch_seeing_eye/Tracking.h"

void callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const stretch_seeing_eye::Tracking::Ptr& msg){
    ROS_DEBUG_STREAM(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, "/camera/color/depth/points", 1);
    message_filters::Subscriber<stretch_seeing_eye::Tracking> info_sub(nh, "/stretch_seeing_eye/face_tracking_point", 1);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, stretch_seeing_eye::Tracking> sync(point_cloud_sub, info_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    /* code */
    return 0;
}
