#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using std::vector;

void loadImages(vector<cv_bridge::CvImage>& arr, vector<std::string> paths) {
    for (int i = 0; i < paths.size(); i++) {
        arr[i] = cv_bridge::CvImage();
        arr[i].image = cv::imread(paths[i]);
        arr[i].encoding = "bgr8";
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // , "/home/nwright/Downloads/photos/Photo2.jpg", "/home/nwright/Downloads/photos/Photo3.jpg", "/home/nwright/Downloads/photos/Photo4.jpg"
    vector<std::string> paths = {"/home/nwright/Downloads/photos/Photo1.jpg"};

    vector<cv_bridge::CvImage> images(paths.size());
    loadImages(images, paths);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/static_image", 1);
    ros::Rate loop_rate(5);

    size_t i = 0;

    while (nh.ok()) {
        pub.publish(images[i].toImageMsg());
        i = (i + 1) % images.size();
        loop_rate.sleep();
    }
}