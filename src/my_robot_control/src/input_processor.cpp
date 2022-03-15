#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "my_robot_msgs/Inputvars.h"

ros::Publisher inputvarspub;

cv::Mat preprocess_frame(cv::Mat frame) {
    // Convert frame to grayscale
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

    // Add Gaussian blur to reduce noise
    cv::GaussianBlur(frame, frame, cv::Size(5,5), 0);

    // Add threshold to make image binary
    cv::threshold(frame, frame, 60, 200, cv::THRESH_BINARY_INV);
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

std::tuple<int, int> calculateInputVars(cv::Mat frame) {
    // Find contours in the frame
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.size() > 0) {
        // Find the contour with the largest area
        int max_index = largest_contour(contours);
        
        // Find the center of the largest contour (x only)
        cv::Moments moment = cv::moments(contours[max_index], false);

        // Find the angle of the largest contour using areaminrect
        cv::RotatedRect rect = cv::minAreaRect(contours[max_index]);

        // Draw the largest contour
        cv::drawContours(frame, contours, max_index, cv::Scalar(255,255,255), 2);

        // Draw the rotated rect
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(frame, vertices[i], vertices[(i+1)%4], cv::Scalar(255,255,255), 2);
        }

        // Show the frame
        cv::imshow("Video Stream", frame);
        cv::waitKey(1);

        if (rect.size.width < rect.size.height) {
            return {moment.m10 / moment.m00, rect.angle};
        } else {
            return  {moment.m10 / moment.m00, rect.angle - 90};
        }
    }
    return {320, 0};
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    frame = preprocess_frame(frame);
    auto [line_center, angle] = calculateInputVars(frame);

    // Calculate the distance between the middle of the frame and the center of the line
    auto message = my_robot_msgs::Inputvars();
    message.error = line_center - frame.cols/2;
    message.angle = angle;
    
    // Make sure the error is within the possible range
    if (message.error <= 320 && message.error >= -320 && message.angle <= 90 && message.angle >= -90) {
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