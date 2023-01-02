#include <ros/ros.h>
#include <thread>
#include <chrono>

// Include message definitions
#include <sensor_msgs/Imu.h>
#include <gabby_msgs/EncoderData.h>

#include "../lib/kalmanfilter.h" // Contains the KalmanFilter class
#include "../lib/measurements.h" // Contains the structs for the measurements

// Declare the needed global variables
KalmanFilter kf;


Vector3d GYRO_STD(0.001243, 0.0055, 0.0045);
Vector3d ACCEL_STD(0.1, 0.05, 0.65); // 0.034
double YAW_STD = 0.00112;
double LIDAR_STD = 0.2; // range std

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, milliseconds, system_clock, seconds

int main(int argc, char **argv) {
    ros::init(argc, argv, "extended_kalman_filter");
    ros::NodeHandle nh;

    // Declare the sensor subscribers
    //ros::Subscriber imu_sub = nh.subscribe("/imu/data", 1, &KalmanFilter::imuCallback, &kf);
    //ros::Subscriber encoder_sub = nh.subscribe("/wheel_odom", 1, &KalmanFilter::encoderCallback, &kf);
    //ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, &KalmanFilter::lidarCallback, &kf);

    // Initialise the filter
    kf = KalmanFilter();
    kf.setYawSTD(YAW_STD);
    kf.setAccelSTD(ACCEL_STD(0));
    kf.setLidarSTD(LIDAR_STD);

    Vector4d state = Vector4d::Zero();
    state(0) = 0.0;
    state(1) = 0.0;
    // 45 degrees to the right of the x-axis in radians yaw
    state(2) = 0.785398;
    state(3) = 1.0;
    kf.setState(state);
    kf.setCovariance(MatrixXd::Identity(4, 4));

    while (ros::ok()) {
        ros::spinOnce();
        // sleep for half a second
        sleep_for(milliseconds(1000));

        // run the prediction step
        kf.predictionStep(1);
        ROS_INFO("Predicted state: %f, %f, %f, %f", kf.getState()(0), kf.getState()(1), kf.getState()(2), kf.getState()(3));
    }

    ros::spin();
 }