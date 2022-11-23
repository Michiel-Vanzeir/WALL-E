#include "ransac.h"

RANSAC::RANSAC(double max_delta, 
               double max_avg_delta,
               double min_line_length, 
               int min_inliers
               ) {
    
    // Initialise the RANSAC parameters
    max_delta = max_delta;
    max_avg_delta = max_avg_delta;
    min_inliers = min_inliers;
    min_line_length = min_line_length;
}

// Calculate the optimal number of iterations
int RANSAC::calculateIterations(int s, int e) {
    // s = sample size, e = estimated prob of a point being an outlier
    int max_iterations = log(1 - 0.99) / log(1 - pow(e, s));
    return max_iterations+3; // Add three iterations to play it safe
}

void RANSAC::setIterations(int optimal_iterations) {
    max_iterations = optimal_iterations;
}

// Extract a line from a point cloud & removes the inliers from it 
Vector2d RANSAC::extractLine(sensor_msgs::PointCloud &cloud) {
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
        double line_length = sqrt(pow(cloud.points[p1].x - cloud.points[p2].x, 2) + pow(cloud.points[p1].y - cloud.points[p2].y, 2));

        // Check if the line is valid & has more outliers than the previous best
        if (inliers >= min_inliers && avg_delta <= max_avg_delta && line_length >= min_line_length && inliers > max_inliers) {
            max_inliers = inliers;
            best_m = m;
            best_c = c;
        }
    }

    // If no line was fitted, return a line with 0 slope
    if (max_inliers == 0) {
        return Vector2d(0, 0);
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

    // Return the best fitted line
    return Vector2d(best_m, best_c);
}

// Extract all landmarks from a point cloud
std::vector<Vector2d> RANSAC::extractLandmarks(sensor_msgs::PointCloud pointcloud) {
    if (pointcloud.points.size() < 2) 
        return std::vector<Vector2d>();

    // Lines fitted is a vector of VectorXd elements
    std::vector<Vector2d> lines_fitted;
    while (true) {
        Vector2d extracted_line = extractLine(pointcloud);

        // If no line was fitted, break
        if (extracted_line[0] == 0 && extracted_line[1] == 0) {
            break;
        }

        lines_fitted.push_back(extracted_line);
    }

    // If lines_fitted is empty, return nothing
    if (lines_fitted.size() == 0) 
        return std::vector<Vector2d>();
    
    return lines_fitted;
}