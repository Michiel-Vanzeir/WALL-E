#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h" 
#include <message_filters/subscriber.h>
#include "laser_geometry/laser_geometry.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "my_robot_msgs/Landmark.h"

#include "../lib/ransac.h"

// Converts laserscans to pointclouds and extracts landmarks from them
class LaserProcessor {
  private:
    ros::NodeHandle nh_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
    
    ros::Subscriber scan_sub_;
    ros::Publisher landmark_pub_;
    RANSAC ransac(3.0, 2.0, 40.0, 6);

  public:
    LaserProcessor() {
      scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &LaserProcessor::scanCallback, this);
      landmark_pub_ = nh_.advertise<my_robot_msgs::Landmark>("/landmarks", 10, false);
      optimal_iterations = ransac.calculateIterations(2, 0.1);
      ransac.setIterations(optimal_iterations);
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

      // Extract landmarks from the pointcloud
      std::vector<Vector2d> landmarks = ransac.extractLandmarks(cloud);

      landmark_pub_.publish(landmarks);
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "landmark_extractor");

  LaserProcessor laser_processor;

  ros::spin();
  return 0;
}