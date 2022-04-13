#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "my_robot_msgs/Inputvars.h"

ros::Publisher inputvarspub;

cv::Mat removeShadows(cv::Mat img) {
    // Split the image into its channels
    std::vector<cv::Mat> rgb_planes;
    cv::split(img, rgb_planes);

    cv::Mat result_planes[3];

    // Normalize each channel
    for (int i = 0; i < 3; i++) {
        cv::Mat plane_result;
        
        cv::dilate(rgb_planes[i], plane_result, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
        cv::medianBlur(plane_result, plane_result, 21);
       
        // cv:absdiff and 255
        cv::Mat abs_diff;
        cv::Mat scalar = cv::Mat(plane_result.size(), CV_8UC1, cv::Scalar(255));
        cv::absdiff(rgb_planes[i], plane_result, abs_diff);
        plane_result = scalar - abs_diff;

        cv::normalize(plane_result, plane_result, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        result_planes[i] = plane_result;
    }

    // Merge the channels
    cv::Mat result;
    cv::merge(result_planes, 3, result);
    return result;
}

cv::Mat preprocess_frame(cv::Mat frame) {
    // Add Gaussian
    cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);

    // Remove the shadows from the frame
    frame = removeShadows(frame);

    // Convert the frame to HSV
    cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

    // Make a mask to binarize the frame to detect the line
    cv::inRange(frame, cv::Scalar(0, 0, 0), cv::Scalar(180, 175, 170), frame);

    // Dilate white pixels in the frame
    cv::dilate(frame, frame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
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

std::tuple<int, int> calculateInputVars(cv::Mat frame, cv::Mat frame2) {
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
        //cv::drawContours(frame2, contours, max_index, cv::Scalar(0,255,0), 2);
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(frame2, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2);
        }
        
        // Rosinfo the vertices
        ROS_INFO("Vertices: %f, %f", vertices[0].x, vertices[1].x);
        if (rect.size.width > rect.size.height) {
            // Put text the angle
            cv::putText(frame2, "right: " + std::to_string(rect.angle+90), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            // Show the frame
            cv::imshow("Mask", frame2);
            cv::waitKey(1);
            return {moment.m10 / moment.m00, rect.angle+90};
        } else {
            // Put text the angle
            cv::putText(frame2, "left: " + std::to_string(rect.angle), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            // Show the frame
            cv::imshow("Mask", frame2);
            cv::waitKey(1);
            return  {moment.m10 / moment.m00, rect.angle};
        }
    }
    return {0, 0};
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat frame2 = frame; 
    frame = preprocess_frame(frame);
    auto [line_center, angle] = calculateInputVars(frame, frame2);

    // Calculate the distance between the middle of the frame and the center of the line
    auto message = my_robot_msgs::Inputvars();
    message.error = line_center - frame.cols/2;
    message.angle = angle;
    // Make sure the error is within the possible range
    //ROS_INFO("Error: %d, Angle: %d", message.error, message.angle);
    if (message.error <= 160 && message.error >= -160 && message.angle <= 90 && message.angle >= -90) {
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
