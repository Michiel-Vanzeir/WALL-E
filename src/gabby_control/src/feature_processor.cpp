#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



#include <sensor_msgs/Image.h>

class FeatureProcessor {
    public:
        FeatureProcessor(ros::NodeHandle& nh) {
            nh_ = nh;
            image_sub_ = nh_.subscribe("/image", 1, &FeatureProcessor::imageCallback, this);
        }
        imageCallback(const sensor_msgs::Image::ConstPtr& image) {
            cv::Mat mat = cv::imdecode(image->data, IMREAD_GRAYSCALE);

            // Apply gaussian with sigma threshold of 0.2
            cv::GaussianBlur(mat, mat, cv::Size(5,5), 1);
            
            // Apply Shi Tomasi corner detection
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(mat, corners, 100, 0.01, 20);

            // Draw the corners
            for (int i = 0; i < corners.size(); i++) {
                cv::circle(mat, corners[i], 5, cv::Scalar(0,0,255), 2);
            }

            // Display the image
            cv::imshow("Image", mat);
            cv::waitKey(1);
        }
    private:
        ros::NodeHandle nh_;
        ros::Subscriber image_sub_;
}