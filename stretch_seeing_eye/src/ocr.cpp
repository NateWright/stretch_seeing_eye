#include <cv_bridge/cv_bridge.h>
#include <leptonica/allheaders.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tesseract/baseapi.h>

void callback(const sensor_msgs::ImageConstPtr &msg) {
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Perform OCR on the image using Tesseract
    tesseract::TessBaseAPI tess;
    tess.Init(NULL, "eng");  // Initialize Tesseract with English language
    tess.SetImage(cv_image->image.data, cv_image->image.cols, cv_image->image.rows, 3, cv_image->image.step);
    tess.

        // Get the OCR result
        std::string extracted_text = tess.GetUTF8Text();

    // Print the extracted text
    ROS_INFO("Extracted Text: %s", extracted_text.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ocr_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/image_publisher_1686711765675924100/image_raw", 1, callback);
    ros::spin();
    return 0;
}