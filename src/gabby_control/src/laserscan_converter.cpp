#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class scanConverter {
    public:
        scanConverter(ros::NodeHandle& nh) {
            nh_ = nh;
            scan_sub_ = nh_.subscribe("/scan", 1, &scanConverter::scanCallback, this);
            image_pub_ = nh_.advertise<sensor_msgs::Image>("/image", 1);
        }
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
            // Convert laserscan to pointcloud
            sensor_msgs::PointCloud2 cloud;
            projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, listener_);
            
            // Convert sensor_msgs::pointcloud to a pcl::PointCloud
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(cloud, pcl_cloud);

            // Convert pcl::PointCloud to cv::Mat
            cv::Mat image(pcl_cloud.height, pcl_cloud.width, CV_32FC3, cv::Scalar::all(0));
            for (int i = 0; i < pcl_cloud.height; i++) {
                for (int j = 0; j < pcl_cloud.width; j++) {
                    image.at<cv::Vec3f>(i, j)[0] = pcl_cloud.points[i * pcl_cloud.width + j].x;
                    image.at<cv::Vec3f>(i, j)[1] = pcl_cloud.points[i * pcl_cloud.width + j].y;
                    image.at<cv::Vec3f>(i, j)[2] = pcl_cloud.points[i * pcl_cloud.width + j].z;
                }
            }

            // Convert  cv::Mat to sensor_msgs::Image
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

            // Publish the image
            image_pub_.publish(msg);
        }
    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Publisher image_pub_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_converter");
    ros::NodeHandle nh;

    scanConverter converter(nh);
    
    ros::spin();
    return 0;
}