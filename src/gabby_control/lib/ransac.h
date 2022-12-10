#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include <eigen3/Eigen/Dense>

#include "sensor_msgs/PointCloud.h"
#include "gabby_msgs/Landmark.h"
#include "gabby_msgs/LandmarkList.h"

using Eigen::Vector2d;
using Eigen::MatrixXd;

class RANSAC {
    private:
        double max_delta;
        double max_avg_delta;
        double min_line_length;
        double max_inlier_dist; 
        int min_inliers;
        int max_iterations;
    
    public:
        RANSAC();
        RANSAC(double max_delta, double max_avg_delta, double min_line_length, double inlier_dist, int min_inliers);

        int calculateIterations(int s, double e);
        void setIterations(int max_iterations);
        double calculateLineLength(Vector2d p1, Vector2d p2);
        Vector2d linearRegression(std::vector<double> x, std::vector<double> y);
        gabby_msgs::Landmark extractLine(sensor_msgs::PointCloud &cloud);
        gabby_msgs::LandmarkList extractLandmarks(sensor_msgs::PointCloud pointcloud);
};

#endif