#include "ros/ros.h"
#include "computer_vision/motor_throttle.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher command_pub;

void videoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the ROS Image to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image;
    // Do some processing on the image and store them in new Mat objects
    cv::cvtColor(cv_ptr->image, image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(image, image, cv::Size(5, 5), 0);
    cv::threshold(image, image, 60, 255, cv::THRESH_BINARY_INV);
    // Find the biggest contour and draw it
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
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
    computer_vision::motor_throttle  motor_msg;
    try {
        int lx = moment.m10 / moment.m00;
        int ly = moment.m01 / moment.m00;

        float std_throttle_left = 0.208;
        float std_throttle_right = 0.272;
        int distance = lx - (image.cols / 2); 
        float extra_throttle = distance*0.0025;

        motor_msg.left_motor = std_throttle_left + extra_throttle;
        motor_msg.right_motor = std_throttle_right - extra_throttle;

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
    command_pub = n.advertise<computer_vision::motor_throttle>("motor_controls", 10);

    ros::spin();
    return 0;
}
