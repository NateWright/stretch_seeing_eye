/**
 * https://itecnote.com/tecnote/c-opencv-c-obj-c-detecting-a-sheet-of-paper-square-detection/
 * https://github.com/alyssaq/opencv
 */
#include <cv_bridge/cv_bridge.h>
#include <leptonica/allheaders.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tesseract/baseapi.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using cv::Mat;
using cv::Point;
using std::vector;

tesseract::TessBaseAPI *ocr;
ros::Publisher pub;

double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void find_squares(Mat &image, vector<vector<Point>> &squares) {
    // blur will enhance edge detection
    Mat blurred(image);
    medianBlur(image, blurred, 9);

    Mat gray0(blurred.size(), CV_8U), gray;
    vector<vector<Point>> contours;

    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++) {
        int ch[] = {c, 0};
        mixChannels(&blurred, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        const int threshold_level = 2;
        for (int l = 0; l < threshold_level; l++) {
            // Use Canny instead of zero threshold level!
            // Canny helps to catch squares with gradient shading
            if (l == 0) {
                Canny(gray0, gray, 10, 20, 3);  //

                // Dilate helps to remove potential holes between edge segments
                dilate(gray, gray, Mat(), Point(-1, -1));
            } else {
                gray = gray0 >= (l + 1) * 255 / threshold_level;
            }

            // Find contours and store them in a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            // Test contours
            vector<Point> approx;
            for (size_t i = 0; i < contours.size(); i++) {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if (approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx))) {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++) {
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if (maxCosine < 0.3)
                        squares.push_back(approx);
                }
            }
        }
    }
}

static void drawSquares(Mat &image, const vector<vector<Point>> &squares) {
    for (size_t i = 0; i < squares.size(); i++) {
        const Point *p = &squares[i][0];

        int n = (int)squares[i].size();
        // dont detect the border
        if (p->x > 3 && p->y > 3)
            cv::polylines(image, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }
}

void pruneSquares(const vector<vector<Point>> &squares, vector<vector<Point>> &pruned_squares) {
    size_t largest_index;
    int largest_area = -1;
    for (size_t i = 0; i < squares.size(); i++) {
        int area = cv::contourArea(squares[i]);
        if (area > largest_area) {
            largest_area = area;
            largest_index = i;
        }
    }
    pruned_squares.push_back(squares[largest_index]);
}

void callback(const sensor_msgs::ImageConstPtr &msg) {
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // vector<vector<Point>> squares, pruned_squares;
    // find_squares(cv_image->image, squares);
    // pruneSquares(squares, pruned_squares);
    // ROS_INFO_STREAM("Found " << pruned_squares.size() << " squares");
    // drawSquares(cv_image->image, squares);
    // cv::Rect roi = cv::boundingRect(pruned_squares[0]);
    // cv::Mat cropped = cv_image->image(roi);
    // cv_image->image = cropped;

    pub.publish(*cv_image->toImageMsg());

    ocr->SetImage(cv_image->image.data, cv_image->image.cols, cv_image->image.rows, 3, cv_image->image.step);

    // Get the OCR result
    std::string extracted_text = std::string(ocr->GetUTF8Text());

    // Print the extracted text
    ROS_INFO("Extracted Text:\n%s", extracted_text.c_str());
}

int main(int argc, char **argv) {
    ocr = new tesseract::TessBaseAPI();
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
    ocr->SetPageSegMode(tesseract::PSM_AUTO);

    ros::init(argc, argv, "ocr_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/static_image", 1, callback);
    pub = nh.advertise<sensor_msgs::Image>("/test_image", 1);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete ocr;
    return 0;
}