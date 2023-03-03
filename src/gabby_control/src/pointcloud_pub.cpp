#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

class scanConverter {
    public:
        scanConverter(ros::NodeHandle& nh) {
            nh_ = nh;
            scan_sub_ = nh_.subscribe("/scan", 1, &scanConverter::scanCallback, this);
            point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointCloud", 1);
        }
        scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
            sensor_msgs::PointCloud2 cloud;
            projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, listener_);
            point_cloud_pub_.publish(cloud);
        }
    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Publisher point_cloud_pub_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_converter");
    ros::NodeHandle nh;

    scanConverter converter(nh);
    
    ros::spin();
    return 0;
}