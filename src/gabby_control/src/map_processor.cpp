#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_processor");
    ros::NodeHandle nh;

    // Subscribe to lidar scans
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, lidarCallback);

    ros::spin();
}