#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include "gabby_msgs/LandmarkList.h"
#include "gabby_msgs/Landmark.h"

#include "../lib/kalmanfilter.h"
#include "../lib/measurements.h"

// Declare the needed global variables
KalmanFilter kf;

constexpr double GYRO_STD_X = 0.001243;
constexpr double GYRO_STD_Y = 0.0055;
constexpr double GYRO_STD_Z = 0.0045;
constexpr double ACCEL_STD_X = 0.034;
constexpr double ACCEL_STD_Y = 0.05;
constexpr double ACCEL_STD_Z = 0.65;
constexpr double LIDAR_STD = 0.2;

void PredictionCallback(const sensor_msgs::Imu::ConstPtr& imu) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    
    // Store all sensor info in a struct
    GyroMeasurement gyro;   
    gyro.psiX = imu->angular_velocity.x;
    gyro.psiZ = imu->angular_velocity.z;
    gyro.psiY = imu->angular_velocity.y;

    // Run the prediction step
    kf.predictionStep(gyro, 0.25);

    // Publish the state
    VectorXd state = kf.getState();
    transform.setOrigin(tf::Vector3(state(0), state(1), state(2)));
    tf::Quaternion q;
    q.setRPY(0, state(3), state(4));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/robot"));
}

void UpdateCallback(const gabby_msgs::LandmarkList::ConstPtr& landmarks) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Store all sensor info in a struct
    LidarMeasurement lidar;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "kalman_filter");
    ros::NodeHandle nh;


    // Initialise the Kalman filter
    Eigen::VectorXd state = VectorXd::Zero(6);
    Eigen::MatrixXd cov = MatrixXd::Identity(6,6);
    Eigen::Vector3d gyro_noise_std(GYRO_STD_X, GYRO_STD_Y, GYRO_STD_Z);
    Eigen::Vector3d accel_noise_std(ACCEL_STD_X, ACCEL_STD_Y, ACCEL_STD_Z);
    
    kf = KalmanFilter(state, cov, gyro_noise_std, accel_noise_std, LIDAR_STD);

    // Subscribe to the IMU and LIDAR topics
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 1, PredictionCallback);
    ros::Subscriber lidar_sub = nh.subscribe("/slam/landmarks", 1, UpdateCallback);
    

    ros::spin();
 }