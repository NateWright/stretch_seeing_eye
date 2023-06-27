#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv) {
    /* code */
    if (argc != 2) {
        std::cout << "Usage: test_pointcloud_pub <pcd_file>" << std::endl;
        return 1;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1)  //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    cloud->header.frame_id = "base_link";

    ros::init(argc, argv, "test_pointcloud_pub");
    ros::NodeHandle nh;
    ros::Publisher testPub = nh.advertise<sensor_msgs::PointCloud2>("/test_pointcloud", 1);
    ros::Publisher testPub2 = nh.advertise<sensor_msgs::Image>("/test_image", 1);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    sensor_msgs::Image image_msg;
    pcl::toROSMsg(*cloud, image_msg);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        testPub.publish(cloud_msg);
        testPub2.publish(image_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
