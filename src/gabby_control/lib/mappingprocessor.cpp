#include "mappingprocessor.h"

MappingProcessor::MappingProcessor() {
    listener_.setExtrapolationLimit(ros::Duration(0.1));
}

// Runs a median filter on the laserscan
sensor_msgs::LaserScan MappingProcessor::medianFilter(const sensor_msgs::LaserScan::ConstPtr& scan) {
    sensor_msgs::LaserScan filtered_scan = *scan;
    int window_size = 5;
    int window_start = 0;
    int window_end = window_size;
    int window_mid = window_size/2;
    int num_ranges = scan->ranges.size();
    std::vector<float> ranges = scan->ranges;
    std::vector<float> filtered_ranges = scan->ranges;
    std::vector<float> window;
    while (window_end < num_ranges) {
        window = std::vector<float>(ranges.begin() + window_start, ranges.begin() + window_end);
        std::sort(window.begin(), window.end());
        filtered_ranges[window_mid] = window[window_mid];
        window_start++;
        window_end++;
        window_mid++;
    }
    filtered_scan.ranges = filtered_ranges;
    return filtered_scan;
}

// Converts a laserscan to a pointcloud
void MappingProcessor::scanToPointcloud(const sensor_msgs::LaserScan::ConstPtr& scan) {
    try {
        projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, listener_);
    } catch (tf::TransformException& e) {
        ROS_ERROR("Could not transform laser scan to pointcloud: %s", e.what());
    }
}

// Casts votes from the pointcloud into the accumulator
void castVotesFromPointcloud() {
    for (int i = 0; i < cloud.points.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;
        float r = sqrt(x*x + y*y);
        float theta = atan2(y, x);
        int r_index = (int) (r * 100);
        int theta_index = (int) (theta * 180 / M_PI);
        accumulator[r_index][theta_index]++;
    }
}