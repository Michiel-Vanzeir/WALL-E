#include "ransac.h"
#include "ros/ros.h"
 #include <numeric>
#include <cmath>

RANSAC::RANSAC() {return;}

RANSAC::RANSAC(double delta, 
               double avg_delta,
               double line_length, 
               double inlier_dist,
               int inliers
               ) {
    
    // Initialise the RANSAC parameters
    max_delta = delta;
    max_avg_delta = avg_delta;
    min_inliers = inliers;
    min_line_length = line_length;
    max_inlier_dist = inlier_dist;
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

double RANSAC::calculateLineLength(Vector2d p1, Vector2d p2) {
    double length = sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
    return length;
}
// Implements linear regression on a set of points
// Returns the slope and y-intercept of the line
Vector2d RANSAC::linearRegression(std::vector<double> x, std::vector<double> y) {
    // Calculate the mean of the x and y values
    double x_mean = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
    double y_mean = std::accumulate(y.begin(), y.end(), 0.0) / y.size();
    
    // Calculate the numerator and denominator of the slope
    double numerator = 0.0;
    double denominator = 0.0;
    for (int i = 0; i < x.size(); i++) {
        numerator += (x[i] - x_mean) * (y[i] - y_mean);
        denominator += pow(x[i] - x_mean, 2);
    }
    
    // Calculate the slope and y-intercept
    double slope = numerator / denominator;
    double y_intercept = y_mean - slope * x_mean;
    
    // Return the slope and y-intercept
    Vector2d line;
    line << slope, y_intercept;
    return line;
}

// Extract a line from a point cloud & removes the inliers from it 
gabby_msgs::Landmark RANSAC::extractLine(sensor_msgs::PointCloud &cloud) {
    int max_inliers = 0;
    double best_m = 0;
    double best_c = 0;
    std::vector<double> best_inliers_x;
    std::vector<double> best_inliers_y;

    for (int i = 0; i < max_iterations; i++) {
        // Randomly select 2 points
        int p1 = rand() % cloud.points.size();
        int p2 = rand() % cloud.points.size();
        while (p1 == p2) {
            p2 = rand() % cloud.points.size();
        }

        // Fit a line through the 2 points
        double m = (cloud.points[p2].y - cloud.points[p1].y) / (cloud.points[p2].x - cloud.points[p1].x);
        double c = cloud.points[p1].y - m * cloud.points[p1].x;

        // Keep track of all inliers
        int inliers = 0;
        std::vector<double> inliers_x;
        std::vector<double> inliers_y;

        // Calculate the distance of each point from the line & save the outer points
        Vector2d outer_p1 = Vector2d(cloud.points[p1].x, cloud.points[p1].y);
        Vector2d outer_p2 = Vector2d(cloud.points[p2].x, cloud.points[p2].y);
        double outer_length = calculateLineLength(outer_p1, outer_p2);
        double avg_delta = 0;
        for (int j = 0; j < cloud.points.size(); j++) {
            double delta = abs(cloud.points[j].y - m * cloud.points[j].x - c);

            // Check if the point is an inlier  
            if (delta <= max_delta) {
                inliers++;
                avg_delta += delta;
                inliers_x.push_back(cloud.points[j].x);
                inliers_y.push_back(cloud.points[j].y);

                // Check if the point is a most outer point
                Vector2d current_point(cloud.points[j].x, cloud.points[j].y);
                double outer1_length = calculateLineLength(outer_p1, current_point);
                double outer2_length = calculateLineLength(outer_p2, current_point);
                if (outer1_length > outer2_length && outer1_length > outer_length) {
                    if (outer2_length <= max_delta) {
                    outer_p2 = current_point;
                    outer_length = outer1_length;
                    m = (outer_p2(1) - outer_p1(1)) / (outer_p2(0) - outer_p1(0));
                    c = outer_p1(1) - m * outer_p1(0);
                    }
                } else if (outer2_length > outer1_length && outer2_length > outer_length) {
                    if (outer1_length <= max_delta) {
                    outer_p1 = current_point;
                    outer_length = outer2_length;
                    m = (outer_p2(1) - outer_p1(1)) / (outer_p2(0) - outer_p1(0));
                    c = outer_p1(1) - m * outer_p1(0);
                    }
                }
            }
            
            // Update max_iterations with the new data
            double e = 1 - (double)inliers / cloud.points.size();
            max_iterations = calculateIterations(2, e);
        }
        avg_delta /= inliers;
        double outer_line_length = calculateLineLength(outer_p1, outer_p2); 

        // Check if the line is valid & has more outliers than the previous best
        if (inliers >= min_inliers && avg_delta <= max_avg_delta && outer_line_length >= min_line_length && inliers > max_inliers) {
            max_inliers = inliers;
            best_m = m;
            best_c = c;
            best_inliers_x = inliers_x;
            best_inliers_y = inliers_y;
        }
    }

    // If no line was fitted, return a line with 0 slope and 0 intercept
    if (max_inliers == 0) {
        gabby_msgs::Landmark landmark;
        landmark.m = 0;
        landmark.c = 0;
        return landmark;
    }
    
    // Use regression to improve the line fit
    // Vector2d optimal_line = linearRegression(best_inliers_x, best_inliers_y);
    // best_m = optimal_line(0);
    // best_c = optimal_line(1);
    
    sensor_msgs::PointCloud new_cloud;
    // Find all outliers and add them to new_cloud
    for (int i = 0; i < cloud.points.size(); i++) {
        double delta = abs(cloud.points[i].y - best_m * cloud.points[i].x - best_c);
        if (delta >= 0.08) { // replaced max_delta by 0.03
            new_cloud.points.push_back(cloud.points[i]);
        }
    }

    // Replace the old cloud with the new one
    cloud = new_cloud;

    // Return the best fitted line as a landmark
    gabby_msgs::Landmark landmark;
    landmark.m = best_m;
    landmark.c = best_c;
    return landmark;
}

// Extract all landmarks from a point cloud
gabby_msgs::LandmarkList RANSAC::extractLandmarks(sensor_msgs::PointCloud pointcloud) {
    gabby_msgs::LandmarkList landmarks;
    if (pointcloud.points.size() < 2) {
        ROS_INFO("Point cloud too small");
        return landmarks;
    }

    while (true) {
        gabby_msgs::Landmark extracted_landmark = extractLine(pointcloud);
        // If no line was fitted, break
        if (extracted_landmark.m == 0 && extracted_landmark.c == 0) {
            break;
        }

        landmarks.landmarks.push_back(extracted_landmark);
    }
    
    return landmarks;
}