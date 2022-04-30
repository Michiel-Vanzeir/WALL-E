#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "my_robot_msgs/Inputvars.h"

ros::Publisher inputvarspub;

cv::Mat preprocess_frame(cv::Mat frame) {
    // Add Gaussian
    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);

    // Convert the frame to HSV
    cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

    // Make a mask to binarize the frame to detect the line
    cv::inRange(frame, cv::Scalar(0, 0, 0), cv::Scalar(180, 175, 170), frame);

    // Dilate white pixels in the frame
    cv::dilate(frame, frame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));

    // Search and draw contours 
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Draw the contours on the frame
    cv::Mat drawing = cv::Mat::zeros(frame.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(255, 255, 255);
        cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
    }

    cv::imshow("frame", drawing);
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

double calculateInputVars(cv::Mat frame, cv::Mat frame2) {
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
    cv::Mat frame2 = frame; 
    frame = preprocess_frame(frame);
    int inputvar = calculateInputVars(frame, frame2);

    // Calculate the distance between the middle of the frame and the center of the line
    auto message = my_robot_msgs::Inputvars();
    message.error = inputvar;

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
