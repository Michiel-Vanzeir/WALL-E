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

    // Do some processing on the image and store them in new Mat objects
    cv::cvtColor(cv_ptr->image, image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(image, image, cv::Size(5, 5), 0);
    cv::threshold(image, image, 60, 255, cv::THRESH_BINARY_INV);
    // Find the biggest contour and draw it
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image.copy(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3);
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
        if (x >= 120) {
            ss << "0|0.395";
            ROS_INFO("Turning left");
        } else if (x <= 50) {
            ss << "0.5|0";
            ROS_INFO("Turning right");
        } else {
            ss << "0.5|0.395";
            ROS_INFO("Going straight");
        }   
        motor_msg.data = ss.str();
        command_pub.publish(motor_msg);
    } catch (int err) {
        ROS_INFO("Error when deciding how to turn");
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
