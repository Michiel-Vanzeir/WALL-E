#include "ransac.h"
#include "ros/ros.h"
 #include <numeric>
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

double calculateLineLength(Vector2d p1, Vector2d p2) {
    double length = sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
    return length;
}
// Implements linear regression on a set of points
// Returns the slope and y-intercept of the line
Vector2d linearRegression(std::vector<int> x, std::vector<int> y) {
    double mean_x = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
    double mean_y = std::accumulate(y.begin(), y.end(), 0.0) / y.size();

    // calculate cross-deviation and deviation about x
    double cross_deviation = 0.0;
    double x_deviation = 0.0;
    for (int i = 0; i < x.size(); i++) {
        cross_deviation += (x[i] - mean_x) * (y[i] - mean_y);
        x_deviation += (x[i] - mean_x) * (x[i] - mean_x);
    }

    // Calculate regression coefficients
    double slope = cross_deviation / x_deviation;
    double y_intercept = mean_y - slope * mean_x;
    return Vector2d(slope, y_intercept);
}

// Extract a line from a point cloud & removes the inliers from it 
my_robot_msgs::Landmark RANSAC::extractLine(sensor_msgs::PointCloud &cloud) {
    int max_inliers = 0;
    double best_m = 0;
    double best_c = 0;

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

        // Keep track of all inliers
        std::vector<int> inliers_x;
        std::vector<int> inliers_y;

        // Calculate the distance of each point from the line & save the outer points
        int inliers = 0;
        double avg_delta = 0;
        Vector2d outer_p1 = Vector2d(cloud.points[p1].x, cloud.points[p1].y);
        Vector2d outer_p2 = Vector2d(cloud.points[p2].x, cloud.points[p2].y);
        double outer_length = calculateLineLength(outer_p1, outer_p2);
        for (int j = 0; j < cloud.points.size(); j++) {
            double delta = abs(cloud.points[j].y - m * cloud.points[j].x - c);
            if (delta < max_delta) {
                inliers++;
                avg_delta += delta;
                inliers_x.push_back(cloud.points[j].x);
                inliers_y.push_back(cloud.points[j].y);

                // Check if the point is a most outer point
                Vector2d current_point(cloud.points[j].x, cloud.points[j].y);
                double outer1_length = calculateLineLength(outer_p1, current_point);
                double outer2_length = calculateLineLength(outer_p2, current_point);
                if (outer1_length > outer2_length && outer1_length > outer_length) {
                    outer_p2 = current_point;
                    outer_length = outer1_length;
                    m = (outer_p2(1) - outer_p1(1)) / (outer_p2(0) - outer_p1(0));
                    c = outer_p1(1) - m * outer_p1(0);
                } else if (outer2_length > outer1_length && outer2_length > outer_length) {
                    outer_p1 = current_point;
                    outer_length = outer2_length;
                    m = (outer_p2(1) - outer_p1(1)) / (outer_p2(0) - outer_p1(0));
                    c = outer_p1(1) - m * outer_p1(0);
                }
            }
            
            // Update max_iterations with the new data
            double e = 1 - (double)inliers / cloud.points.size();
            max_iterations = calculateIterations(2, e);
        }
        avg_delta /= inliers;
        double outer_line_length = calculateLineLength(outer_p1, outer_p2); 

        // Check if the line is valid & has more outliers than the previous best
        if (inliers >= min_inliers && avg_delta <= max_avg_delta && line_length >= min_line_length && inliers > max_inliers) {
            max_inliers = inliers;
            best_m = m;
            best_c = c;
        }
    }

    // If no line was fitted, return a line with 0 slope and 0 intercept
    if (max_inliers == 0) {
        my_robot_msgs::Landmark landmark;
        landmark.m = 0;
        landmark.c = 0;
        return landmark;
    }
    
    // Use regression to improve the line fit
    Vector2d optimal_line = linearRegression(inliers_x, inliers_y);
    best_m = optimal_line(0);
    best_c = optimal_line(1);
    
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