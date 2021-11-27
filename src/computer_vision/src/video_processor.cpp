#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "computer_vision/motor_throttle.h"
#include "sensor_msgs/Image.h"

ros::Publisher  motor_throttle_pub;
float prvs_error = 0;
float integral_error = 0;

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
        float std_throttle_left = 0.240;
        float std_throttle_right = 0.313846;

        int error = (lx - (frame.cols / 2)); 
        integral_error += error;
        float derivate = error - prvs_error;
        prvs_error = error;

        // Implementing a PID controller
        float PIDValue = (0.0030*error) + (0.0001*integral_error) + (0.001*derivate);
        motor_msg.left_motor = std_throttle_left + PIDValue;
        motor_msg.right_motor = std_throttle_right - PIDValue;
        ROS_INFO("Left motor value: %f\nPID value: %f", motor_msg.left_motor, PIDValue);

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
