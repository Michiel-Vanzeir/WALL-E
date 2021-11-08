#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>

ros::Publisher command_pub;

void videoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the ROS Image to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    // Create three Mat objects
    cv::Mat gray_image;
    cv::Mat blurred_image;
    cv::Mat threshold_image;

    // Do some processing on the image and store them in new Mat objects
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 0);
    cv::threshold(blurred_image, threshold_image, 45, 255, cv::THRESH_BINARY_INV);
    
    // Find the biggest contour and draw it
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(threshold_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat::zeros(gray_image.size(), CV_8UC3);
    cv::Scalar color = cv::Scalar(255, 255, 255);
    double max_area = 0;
    int max_index = 0;
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_index = i;
        }
    }
 
    // Find the center of the biggest contour and draw a circle 
    cv::Moments moment = cv::moments(contours[max_index], false);
    std_msgs::String motor_msg;
    try {
        int x = moment.m10 / moment.m00;
        int y = moment.m01 / moment.m00;
    
        // Decide whether the robot should turn left or right or go straight
        std::stringstream ss;
        if (x <= drawing.cols / 3) {
            ss << "-0.05|0";
            ROS_INFO("Turning left");
        } else if (x >= drawing.cols / 3) {
            ss << "0|-0.05";
            ROS_INFO("Turning right");
        } else {
            ss << "0.05|0.05";
            ROS_INFO("Going straight");
        }   
        motor_msg.data = ss.str();
        command_pub.publish(motor_msg);
    } catch (int exc) {
        ROS_INFO("Error in videoprocessor.cpp");
}
}

int main(int argc, char **argv) {
    // Initialize the node, nodehandle, subscriber and define the publisher
    ros::init(argc, argv, "video_processor");
    ros::NodeHandle n;
    ros::Subscriber video_sub = n.subscribe("video_feed", 10, videoCallback); 
    command_pub = n.advertise<std_msgs::String>("motor_controls", 10);

    ros::spin();
    return 0;
}
