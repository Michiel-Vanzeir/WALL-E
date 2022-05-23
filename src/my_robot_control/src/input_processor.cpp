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

    // Convert to grayscale
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    // Threshold
    cv::threshold(frame, frame, 80, 255, cv::THRESH_BINARY_INV);

    cv::imshow("Processed Frame", frame);
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
    // Calculate the middle of the area of the line on the x-axis
    int inputvar = calculateInputVars(frame);

    // Calculate the distance between the middle of the frame and middle of the line area
    auto message = my_robot_msgs::Inputvars();
    message.error = inputvar - (frame.cols / 2);

    // Make sure the error is within the possible range
    if (message.error <= 160 && message.error >= -160) {    
        inputvarspub.publish(message);
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "input_processor");
    ros::NodeHandle nh;

    inputvarspub = nh.advertise<my_robot_msgs::Inputvars>("inputfeed", 2);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("videofeed", 2, imageCallback);

    ros::spin();
}
