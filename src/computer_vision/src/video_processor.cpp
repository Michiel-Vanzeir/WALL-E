#include "ros/ros.h"
#include "computer_vision/motor_cmd.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Image.h"

#include <sstream>

void videoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat gray_image;
    cv::Mat blurred_image;
    cv::Mat threshold_image;

    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 0);
    cv::threshold(blurred_image, threshold_image, 25, 255, cv::THRESH_BINARY);
    
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
    cv::drawContours(drawing, contours, max_index, color, 2, 8, hierarchy, 0, cv::Point());

    // Find the center of the biggest contour and draw a circle 
    cv::Moments moment = cv::moments(contours[max_index], false);
    ROS_INFO("ENTERING TRY STATEMENT");
    try {
        int x = moment.m10 / moment.m00;
        int y = moment.m01 / moment.m00;
        cv::circle(drawing, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
    
        // Decide whether the robot should turn left or right or go straight
        ros::NodeHandle n;
        ros::Publisher command_pub = n.advertise<std_msgs::String>("motor_controls", 100);
        std_msgs::String  msg;
        std::stringstream ss;
        if (x < drawing.cols / 2) {
            ss << "-0.5 | 0.5";
            msg.data =  ss.str();
        } else if (x > drawing.cols / 2) {
            ss << "0.5 | -0.5";
            msg.data = ss.str();
        } else {
            ss << "0.5 | 0.5";
            msg.data = ss.str();
        }   
        ROS_INFO("%s", msg.data.c_str());
        command_pub.publish(msg);
        ROS_INFO("Published msg");
        ros::spinOnce();
    } catch (int exc) {
        ROS_INFO("Error in videoprocessor.cpp");
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "video_processor");
    ros::NodeHandle n;
    ros::Subscriber video_sub = n.subscribe("video_feed",30, videoCallback); 
    ROS_INFO("ROS INITLIAZED");
    ros::spin();
    return 0;
}
