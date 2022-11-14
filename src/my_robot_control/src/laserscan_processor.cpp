#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h" 
#include <message_filters/subscriber.h>
#include "laser_geometry/laser_geometry.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

class LaserProcessor {
  public:
    ros::NodeHandle nh_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_(nh, "/scan", 1);
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher cloud_pub_;

    LaserProcessor() : 
      laser_notifier_(laser_sub_, listener_, "/base_link", 10) {
      laser_notifier_.registerCallback(boost::bind(&LaserProcessor::scanCallback, this, _1));
      laser_notifier_.setTolerance(ros::Duration(1));
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/cloud", 10);
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
      ROS_INFO("scanCallback...");
      sensor_msgs::PointCloud cloud;
      try {
        projector_.transformLaserScanToPointCloud("map", *scan_in, cloud, listener_);
      }
      catch (tf::TransformException& e) {
        ROS_ERROR("Received an exception trying to transform a laser scan: %s", e.what());
        return;
      }
      cloud_pub_.publish(cloud);
    }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_processor");
  ros::NodeHandle nh;

  LaserProcessor lproc(nh);

  ros::spin();
  return 0;
}