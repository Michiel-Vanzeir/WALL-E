#include "kalmanfilter.h"
#include "ros/ros.h"
#include "measurements.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Wraps angles to [-pi, pi]
double wrapAngle(double angle) {
    angle = fmod(angle, (2.0*M_PI));
    if (angle <= -M_PI){angle += (2.0*M_PI);}
    else if (angle > M_PI){angle -= (2.0*M_PI);}
    return angle;
}

// Constructor (exists for overloading)
KalmanFilter::KalmanFilter() {
    initialized_ = false;
}

// Constructor for KalmanFilter class
KalmanFilter::KalmanFilter(Vector4d init_state, Matrix4d init_cov, double yaw_std, double acccel_std, double lidar_std) {
    yaw_noise_std = yaw_std;
    accel_noise_std = acccel_std;
    lidar_noise_std = lidar_std;
    
    setState(init_state);
    setCovariance(init_cov);

    initialized_ = true;
}

void KalmanFilter::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Get the yaw angle from the quaternion and save it in the buffer
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    // Get the yaw
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // Wrap the yaw angle to [-pi, pi]
    yaw = wrapAngle(yaw);
    // Store the yaw in the buffer
    gyro_buffer.yaw = yaw;

    // Store accelerometer measurements in buffer
    accel_buffer.acceleration = msg->linear_acceleration.x;
}

void KalmanFilter::encoderCallback(const gabby_msgs::EncoderData::ConstPtr& msg) {
    // Store encoder measurements in buffer
    encoder_buffer.left_distance = msg->left_distance;
    encoder_buffer.right_distance = msg->right_distance;    
}

void KalmanFilter::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Store lidar measurements in buffer
    lidar_buffer.ranges = msg->ranges;
}

// Kalman filter prediction step (uses gyro measurements & wheel encoder measurements (left & right revolutions))
void KalmanFilter::predictionStep(double dt) {
    if (!isInitialised()) {
        return;
    }

    // Declare the vector F
    Vector4d F;
    F << state_(3)*cos(state_(2)), state_(3)*sin(state_(2)), gyro_buffer.yaw, accel_buffer.acceleration;

    // Rosinfo the F vector
    //ROS_INFO("F: %f, %f, yaw: %f, accel: %f", F(0), F(1), F(2), F(3));

    // Compute the jacobean matrix
    Matrix4d J; 
    J << 1, 0, -state_(3)*sin(state_(2)), cos(state_(3)),
         0, 1, state_(3)*cos(state_(2)), sin(state_(3)),
         0, 0, 1, 0,
         0, 0, 0, 1;

    // Compute the covariance matrix
    Matrix4d Q;
    Q << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, pow(dt*yaw_noise_std, 2), 0,
         0, 0, 0, pow(dt*accel_noise_std, 2);
    
    // Update the state and covariance
    Vector4d temp = state_ + dt*F;
    state_ = temp;
    covariance_ = J*covariance_*J.transpose() + Q;
}

// Kalman Update step (uses Lidar, Accelerometer & odometry)
void KalmanFilter::updateStep() {
    int x = 5;
}
