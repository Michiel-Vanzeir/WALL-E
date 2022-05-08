#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "my_robot_msgs/Inputvars.h"

ros::Publisher inputvarspub;

cv::Mat removeShadows(cv::Mat img) {
    // Split the image into its channels
    std::vector<cv::Mat> rgb_planes;
    cv::split(img, rgb_planes);

    cv::Mat result_planes[3];

    // Normalize each channel
    for (int i = 0; i < 3; i++) {
        cv::Mat plane_result;
        
        cv::dilate(rgb_planes[i], plane_result, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
        cv::medianBlur(plane_result, plane_result, 21);
       
        // cv:absdiff and 255
        cv::Mat abs_diff;
        cv::Mat scalar = cv::Mat(plane_result.size(), CV_8UC1, cv::Scalar(255));
        cv::absdiff(rgb_planes[i], plane_result, abs_diff);
        plane_result = scalar - abs_diff;

        cv::normalize(plane_result, plane_result, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        result_planes[i] = plane_result;
    }

    // Merge the channels
    cv::Mat result;
    cv::merge(result_planes, 3, result);
    return result;
}

cv::Mat preprocess_frame(cv::Mat frame) {
    // Add Gaussian
    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);

    // Remove shadows
    // frame = removeShadows(frame);

    // Convert to grayscale
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    // Threshold
    cv::threshold(frame, frame, 60, 255, cv::THRESH_BINARY_INV);

    cv::imshow("frame", frame);
    cv::waitKey(3);

    return frame;
}

int largest_contour(std::vector<std::vector<cv::Point>> contours) {
    // Find the index of the contour with the largest area
    int max_area_index = 0;
    double max_area = 0;
    for (long unsigned int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_area_index = i;
        }
    }
    return max_area_index;
}

double calculateInputVars(cv::Mat frame) {
    // Find contours in the frame
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.size() > 0) {
        // Find the contour with the largest area
        int max_index = largest_contour(contours);
        
        // Find the center of the largest contour (x only)
        cv::Moments moment = cv::moments(contours[max_index], false);

        return moment.m10 / moment.m00;
    }
    return 0.0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    frame = preprocess_frame(frame);
    int inputvar = calculateInputVars(frame);

    // Calculate the distance between the middle of the frame and the center of the line
    auto message = my_robot_msgs::Inputvars();
    message.error = inputvar - (frame.cols / 2);

    // Make sure the error is within the possible range
    if (message.error <= 160 && message.error >= -160) {
        //ROS_INFO("Error: %d, Angle: %d", message.error, message.angle);
        inputvarspub.publish(message);
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "input_processor");
    ros::NodeHandle nh;

    inputvarspub = nh.advertise<my_robot_msgs::Inputvars>("inputfeed", 1);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("videofeed", 1, imageCallback);

    ros::spin();
}
