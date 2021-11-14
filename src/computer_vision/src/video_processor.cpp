#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "computer_vision/motor_throttle.h"
#include "sensor_msgs/Image.h"

ros::Publisher  motor_throttle_pub;

void videoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the ROS Image to an OpenCV frame
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;

    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);
    cv::threshold(frame, frame, 60, 255, cv::THRESH_BINARY_INV);

    // Find the biggest contour and draw it
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    double max_area = 0;
    int max_index = 0;
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_index = i;
        }
    }
 
    // Find the center of the biggest contour and decide the moto throttle value
    cv::Moments moment = cv::moments(contours[max_index], false);
    computer_vision::motor_throttle  motor_msg;
    try {
        int lx = moment.m10 / moment.m00;

        // Motor speeds are imabalanced, so these are the values for going straight
        float std_throttle_left = 0.208;
        float std_throttle_right = 0.272;

        // Find the distance between the center of the biggest contour and the center of the image and decide the extra throttle value
        float extra_throttle = (lx - (frame.cols / 2))*0.0025; 

        motor_msg.left_motor = std_throttle_left + extra_throttle;
        motor_msg.right_motor = std_throttle_right - (extra_throttle*1.30769230769);

        motor_throttle_pub.publish(motor_msg);
    } catch (int err) {
        ROS_INFO("Error when deciding how to turn");
}
}

int main(int argc, char **argv) {
    // Initialize the node, nodehandle, subscriber and define the publisher
    ros::init(argc, argv, "video_processor");
    ros::NodeHandle nh;
    ros::Subscriber video_sub = nh.subscribe("video_feed", 2, videoCallback); 
    motor_throttle_pub = nh.advertise<computer_vision::motor_throttle>("throttle_feed", 2);

    ros::spin();
    return 0;
}
