#include <curl/curl.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

using namespace std;

float prvs_error, PIDintegral = 0;

void postThrottle(float left_motor, float right_motor) { 
    CURL *curl;
    curl = curl_easy_init();

    if (curl) {
        string url = "http://localhost:8080/motor_throttle?sender=video_processor&left_motor=" + to_string(left_motor) + "&right_motor=" + to_string(right_motor);
        
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
}

void videoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the ROS Image to an OpenCV frame
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;

    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);
    cv::threshold(frame, frame, 35, 255, cv::THRESH_BINARY_INV);

    // Look for contours in the frame
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest contour if there are any contours
    if (contours.size() > 0) {
        double max_area = 0;
        int max_index = 0;
        for (int i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_index = i;
            }
        }
    
        // Find the center of the biggest contour
        cv::Moments moment = cv::moments(contours[max_index], false);
        int lx = moment.m10 / moment.m00;

        // Default throttle for going straight
        float std_throttle_left = 0.4;
        float std_throttle_right = 0.385;

        float error = lx - frame.cols/2;
        PIDintegral += error;

        float PIDderivative = error - prvs_error;
        prvs_error = error;

        float speedP = 0.0022 * abs(error);
        float anglePID = 0.625 * error;

        float left_motor = (std_throttle_left - speedP*1.038961) + (((std_throttle_left - speedP*1.038961)*(anglePID/100))/2)*1.038961;
        float right_motor = (std_throttle_right - speedP) - ((std_throttle_left - speedP)*(anglePID/100))/2;

        postThrottle(left_motor, right_motor);
        // ROS_INFO("\nMotors: left=%f, right=%f \nP speed: %f \nPID angle: %f\n", left_motor, right_motor, speedP, anglePID);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "video_processor");
    ros::NodeHandle nh;
    ros::Subscriber video_sub = nh.subscribe("video_feed", 2, videoCallback); 

    ros::spin();
    return 0;
}
