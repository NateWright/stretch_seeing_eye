#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <iostream>

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

void callback(sensor_msgs::PointCloud2 cloud) {
    try {
        sensor_msgs::PointCloud2 transformed_cloud;
        tfBuffer.transform(cloud, transformed_cloud, "base_link", ros::Duration(1.0));

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

        pcl::io::savePCDFileASCII("/home/nwright/ros/test_pcd.pcd", *pcl_cloud);
        ROS_INFO("Saved %d data points to test_pcd.pcd.", (int)pcl_cloud->points.size());
        exit(0);
    } catch (...) {
    }
}

int main(int argc, char **argv) {
    /* code */
    ros::init(argc, argv, "save_pointcloud");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 10, callback);
    std::cout << "starting" << std::endl;
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    ros::spin();
    return 0;
}
