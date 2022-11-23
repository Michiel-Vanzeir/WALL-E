#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include <eigen3/Eigen/Dense>

#include "sensor_msgs/PointCloud.h"

using Eigen::Vector2d;
using Eigen::MatrixXd;

class RANSAC {
    private:
        double max_delta;
        double max_avg_delta;
        double min_line_length; 
        int min_inliers;
        int max_iterations;
    
    public:
        RANSAC(double max_delta, double max_avg_delta, double min_line_length, int min_inliers);

        int calculateIterations(int s, int e);
        void setIterations(int max_iterations);
        Vector2d extractLine(sensor_msgs::PointCloud &cloud);
        std::vector<Vector2d> extractLandmarks(sensor_msgs::PointCloud pointcloud);
};

#endif