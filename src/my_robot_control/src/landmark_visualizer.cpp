#include "ros/ros.h"
#include <cmath>

#include "visualization_msgs/Marker.h"
#include "my_robot_msgs/Landmark.h"
#include "my_robot_msgs/LandmarkList.h"

ros::Publisher marker_pub;

void landmarkCallback(const my_robot_msgs::LandmarkList::ConstPtr& landmarklist) {
    // Create a marker for each landmark, landmarks are lines with m and c
    for (int i = 0; i < landmarklist->landmarks.size(); i++) {
        my_robot_msgs::Landmark landmark = landmarklist->landmarks[i];
        visualization_msgs::Marker marker;
        marker.header.frame_id = "robot";
        marker.header.stamp = ros::Time();
        marker.ns = "landmarks";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.5;
        
        // Find two points on the line with the equation y = mx + c
        geometry_msgs::Point p1;
        p1.x = -10;
        p1.y = landmark.m * p1.x + landmark.c;
        geometry_msgs::Point p2;
        p2.x = 10;
        p2.y = landmark.m * p2.x + landmark.c;


        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker_pub.publish(marker);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "landmark_visualizer");
    ros::NodeHandle nh;
    
    ros::Subscriber landmark_sub = nh.subscribe<my_robot_msgs::LandmarkList>("/slam/landmarks", 10, landmarkCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::spin();
    return 0;
}