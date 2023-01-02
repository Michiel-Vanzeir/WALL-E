#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Dense>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class MappingProcessor {
    public:
        MappingProcessor();

        sensor_msgs::LaserScan medianFilter(const sensor_msgs::LaserScan::ConstPtr& scan);
        void scanToPointcloud(const sensor_msgs::LaserScan::ConstPtr& scan)


    private:
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

        sensor_msgs::PointCloud cloud;
        int accumulator[1400][360];
};


#endif