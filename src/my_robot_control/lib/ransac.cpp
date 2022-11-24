#include "ransac.h"
#include "ros/ros.h"
#include <cmath>

RANSAC::RANSAC() {return;}

RANSAC::RANSAC(double delta, 
               double avg_delta,
               double line_length, 
               int inliers
               ) {
    
    // Initialise the RANSAC parameters
    max_delta = delta;
    max_avg_delta = avg_delta;
    min_inliers = inliers;
    min_line_length = line_length;
}

// Calculate the optimal number of iterations
int RANSAC::calculateIterations(int s, double e) {
    // s = sample size, e = estimated prob of a point being an outlier
    int max_iterations = log(1 - 0.99) / log(1 - pow((1 - e),s));
    return max_iterations;
}

void RANSAC::setIterations(int optimal_iterations) {
    max_iterations = optimal_iterations;
}

// Extract a line from a point cloud & removes the inliers from it 
my_robot_msgs::Landmark RANSAC::extractLine(sensor_msgs::PointCloud &cloud) {
    int max_inliers = 0;
    double best_m = 0;
    double best_c = 0;
    int best_p1 = 0;
    int best_p2 = 0;
    
    for (int i = 0; i < max_iterations; i++) {
        // Randomly select 2 points
        int p1 = rand() % cloud.points.size();
        int p2 = rand() % cloud.points.size();
        while (p1 == p2) {
            p2 = rand() % cloud.points.size();
        }

        // Calculate the line equation
        double m = (cloud.points[p2].y - cloud.points[p1].y) / (cloud.points[p2].x - cloud.points[p1].x);
        double c = cloud.points[p1].y - m * cloud.points[p1].x;

        // Calculate the distance of each point from the line
        int inliers = 0;
        double avg_delta = 0;
        for (int j = 0; j < cloud.points.size(); j++) {
            double delta = abs(cloud.points[j].y - m * cloud.points[j].x - c);
            if (delta < max_delta) {
                inliers++;
                avg_delta += delta;
            }
        }
        avg_delta /= inliers;
        // IMPROVE THIS??
        double line_length = sqrt(pow(cloud.points[p1].x - cloud.points[p2].x, 2) + pow(cloud.points[p1].y - cloud.points[p2].y, 2));

        // Check if the line is valid & has more outliers than the previous best
        if (inliers >= min_inliers && avg_delta <= max_avg_delta && line_length >= min_line_length && inliers > max_inliers) {
            max_inliers = inliers;
            best_m = m;
            best_c = c;
            
            // ros info inliers, avg_delta, line_length, and delta
            ROS_INFO("Inliers: %d, Avg Delta: %f, Line Length: %f", inliers, avg_delta, line_length);
        }
    }

    // If no line was fitted, return a line with 0 slope and 0 intercept
    if (max_inliers == 0) {
        my_robot_msgs::Landmark landmark;
        landmark.m = 0;
        landmark.c = 0;
        return landmark;
    }
    
    

    
    

    sensor_msgs::PointCloud new_cloud;
    // Find all outliers and add them to new_cloud
    for (int i = 0; i < cloud.points.size(); i++) {
        double delta = abs(cloud.points[i].y - best_m * cloud.points[i].x - best_c);
        if (delta >= max_delta) {
            new_cloud.points.push_back(cloud.points[i]);
        }
    }

    // Replace the old cloud with the new one
    cloud = new_cloud;

    // Return the best fitted line as a landmark
    my_robot_msgs::Landmark landmark;
    landmark.m = best_m;
    landmark.c = best_c;
    return landmark;
}

// Extract all landmarks from a point cloud
my_robot_msgs::LandmarkList RANSAC::extractLandmarks(sensor_msgs::PointCloud pointcloud) {
    my_robot_msgs::LandmarkList landmarks;
    if (pointcloud.points.size() < 2) {
        ROS_INFO("Point cloud too small");
        return landmarks;
    }

    while (true) {
        my_robot_msgs::Landmark extracted_landmark = extractLine(pointcloud);
        // If no line was fitted, break
        if (extracted_landmark.m == 0 && extracted_landmark.c == 0) {
            break;
        }

        landmarks.landmarks.push_back(extracted_landmark);
    }
    
    return landmarks;
}