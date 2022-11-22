#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <tf/transform_broadcaster.h>

#include "../lib/kalmanfilter.h"
#include "../lib/measurements.h"

// Declare the needed global variables
ros::Publisher statepub;
KalmanFilter kf;

constexpr double GYRO_STD_X = 0.001243;
constexpr double GYRO_STD_Y = 0.0055;
constexpr double GYRO_STD_Z = 0.0045;
constexpr double ACCEL_STD_X = 0.034;
constexpr double ACCEL_STD_Y = 0.05;
constexpr double ACCEL_STD_Z = 0.65;
constexpr double LIDAR_STD = 0.2;

void KalmanCallback(const sensor_msgs::Imu::ConstPtr& imu, const my_robot_msgs::LineSegmentList::ConstPtr& landmarks) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    
    // Store all sensor info in structs
    GyroMeasurement gyro;   
    gyro.psiX = imu->angular_velocity.x;
    gyro.psiZ = imu->angular_velocity.z;
    gyro.psiY = imu->angular_velocity.y;

    AccelerometerMeasurement accel;
    accel.accelX = imu->linear_acceleration.x;
    accel.accelY = imu->linear_acceleration.y;
    accel.accelZ = imu->linear_acceleration.z;

    LidarMeasurement lidar;
    lidar.angle_min = scan->angle_min;
    lidar.angle_max = scan->angle_max;
    lidar.angle_increment = scan->angle_increment;
    lidar.range_min = scan->range_min;
    lidar.range_max = scan->range_max;
    lidar.ranges = scan->ranges;

    // Run the prediction step
    kf.predictionStep(gyro, 0.5);

    // Publish the state
    VectorXd state = kf.getState();

    transform.setOrigin(tf::Vector3(state(0), state(1), state(2)));
    tf::Quaternion q;
    q.setRPY(0, state(3), state(4));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/robot"));
    
    // Iterate through landmarks
    foreach (my_robot_msgs::LineSegment line, landmarks->segments) {
        slope = (line.end.y - line.start.y) / (line.end.x - line.start.x);
        
    }

    // Run the update step
    kf.updateStep(accel, lidar);
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
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data", 2);
    message_filters::Subscriber<my_robot_msgs::LineSegmentList> landmark_sub(nh, "/lidar/line_segments", 2);

    // Synchronise the IMU and LIDAR topics
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, my_robot_msgs::LineSegmentList> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, lidar_sub);
    sync.registerCallback(boost::bind(&KalmanCallback, _1, _2));

    ros::spin();
 }