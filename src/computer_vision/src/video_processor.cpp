#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Image.h"

static const std::string OPENCV_WINDOW = "opencv window";

void videoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    ROS_INFO("I received something");
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "video_processor");
    ros::NodeHandle n;
    ros::Subscriber video_sub = n.subscribe("video_feed",30, videoCallback); 
    ROS_INFO("ROS INITLIAZED");
    ros::spin();
    return 0;
}
