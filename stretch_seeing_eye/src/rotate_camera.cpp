#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class RotateCamera {
   private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber depthSub_;
    ros::Subscriber imageSub_;
    ros::Subscriber depthInfoSub_;
    ros::Subscriber imageInfoSub_;

    ros::Publisher depthPub_;
    ros::Publisher imagePub_;
    ros::Publisher depthInfoPub_;
    ros::Publisher imageInfoPub_;

    tf2_ros::TransformListener *tfListener_;
    tf2_ros::Buffer tfBuffer_;

   public:
    explicit RotateCamera(ros::NodeHandlePtr nh) : nh_(nh) {
        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

        depthSub_ = nh->subscribe("/camera/aligned_depth_to_color/image_raw", 10, RotateCamera::depth_callback, this);
        imageSub_ = nh->subscribe("/camera/color/image_raw", 10, RotateCamera::image_callback, this);
        depthInfoSub_ = nh->subscribe("/camera/aligned_depth_to_color/camera_info", 10, RotateCamera::depth_info_callback, this);
        imageInfoSub_ = nh->subscribe("/camera/color/camera_info", 10, RotateCamera::image_info_callback, this);

        depthPub_ = nh->advertise<sensor_msgs::PointCloud2>("/camera/aligned_depth_to_color/image_raw_rotated", 10);
        imagePub_ = nh->advertise<sensor_msgs::Image>("/camera/color/image_raw_rotated", 10);
        depthInfoPub_ = nh->advertise<sensor_msgs::CameraInfo>("/camera/aligned_depth_to_color/camera_info_rotated", 10);
        imageInfoPub_ = nh->advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info_rotated", 10);
    }
    ~RotateCamera() {
        delete tfListener_;
    }
    void depth_callback(const sensor_msgs::PointCloud2::Ptr msg) {
        auto output = tfBuffer_.transform(*msg, "base_link");
        depthPub_.publish(output);
    }
    void image_callback(const sensor_msgs::Image::Ptr msg) {
        auto output = tfBuffer_.transform(*msg, "base_link");
        imagePub_.publish(output);
    }
    void depth_info_callback(const sensor_msgs::CameraInfo::Ptr msg) {
        auto output = tfBuffer_.transform(*msg, "base_link");
        depthInfoPub_.publish(output);
    }
    void image_info_callback(const sensor_msgs::CameraInfo::Ptr msg) {
        auto output = tfBuffer_.transform(*msg, "base_link");
        imageInfoPub_.publish(output);
    }
};

int main(int argc, char **argv) {
    /* code */
    ros::init(argc, argv, "rotate_camera");
    ros::NodeHandlePtr nh(new ros::NodeHandle());
    RotateCamera rotate_camera;
    ros::spin();
    return 0;
}
