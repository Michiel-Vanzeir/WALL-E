#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <gabby_msgs/Features.h>
#include <geometry_msgs/Point.h>


class FeatureProcessor {
    public:
        FeatureProcessor(ros::NodeHandle& nh) {
            nh_ = nh;
            image_sub_ = nh_.subscribe("/image", 1, &FeatureProcessor::imageCallback, this);
            image_pub_ = nh_.advertise<sensor_msgs::Image>("/cornerImage", 1);
            feature_pub_ = nh_.advertise<gabby_msgs::Features>("/features", 1);
        }
        void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
            cv::Mat mat = cv::imdecode(image->data, cv::IMREAD_GRAYSCALE);

            // Apply gaussian with sigma threshold of 0.2
            cv::GaussianBlur(mat, mat, cv::Size(5,5), 1);
            
            // Apply Shi Tomasi corner detection
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(mat, corners, 100, 0.01, 20);

            // Draw the corners
            for (int i = 0; i < corners.size(); i++) {
                cv::circle(mat, corners[i], 5, cv::Scalar(0,0,255), 2);
            }
            //  Common descriptors include SIFT, SURF, ORB, and others.
            // Convert cv::Mat to sensor_msgs::Image
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat).toImageMsg();

            // Publish the features
            gabby_msgs::Features features;
            for (int i = 0; i < corners.size(); i++) {
                geometry_msgs::Point point;
                point.x = corners[i].x;
                point.y = corners[i].y;
                point.z = 0;
                features.points.push_back(point);
            }
            feature_pub_.publish(features);

            // Publish the image
            image_pub_.publish(msg);
        }
    private:
        ros::NodeHandle nh_;
        ros::Subscriber image_sub_;
        ros::Publisher image_pub_;
        ros::Publisher feature_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_processor");
    ros::NodeHandle nh;
    FeatureProcessor fp(nh);
    ros::spin();
    return 0;
}