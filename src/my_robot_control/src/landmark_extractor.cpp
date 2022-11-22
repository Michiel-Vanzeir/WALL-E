#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h" 
#include <message_filters/subscriber.h>
#include "laser_geometry/laser_geometry.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

// Converts laserscans to pointclouds and extracts landmarks from them
class LaserProcessor {
  private:
    ros::NodeHandle nh_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
    
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;

  public:
    LaserProcessor() {
      scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &LaserProcessor::scanCallback, this);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/pointcloud", 10, false);
    }

    void extractLandmarks(const sensor_msgs::PointCloud cloud) {
      
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
      sensor_msgs::PointCloud cloud;
      try {
        projector_.transformLaserScanToPointCloud("world", *scan_in, cloud, tfListener_);
      }
      catch (tf::TransformException& e) {
        ROS_ERROR("Received an exception trying to transform a laser scan: %s", e.what());
        return;
      }

      extractLandmarks(cloud);
      cloud_pub_.publish(cloud);
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_processor");

  LaserProcessor laser_processor;

  ros::spin();
  return 0;
}