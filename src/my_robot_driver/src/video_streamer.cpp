#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_streamer");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("videofeed", 2);

    // Configure the video capture
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    if (!cap.isOpened()) {
        ROS_ERROR("Could not open video stream...");
        return -1;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(20); // 20 Hz
    while (nh.ok()) {
        // Capture a frame
        cap >> frame;

        if (!frame.empty()) {
            // Convert the frame to a ROS message and publish it
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  }
