#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
 #include <geometry_msgs/TransformStamped.h>

#include "../lib/kalmanfilter.h"
#include "../lib/sensors.h" // can be removed

ros::Publisher statepub;
KalmanFilter kf;

void KalmanCallback(const sensor_msgs::Imu::ConstPtr& imu, const sensor_msgs::LaserScan::ConstPtr& scan) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "/baselink";
    
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
    transformStamped.transform.translation.x = state(0);
    transformStamped.transform.translation.y = state(1);
    transformStamped.transform.translation.z = state(2);
    tf2::Quaternion q;
    q.setRPY(0, state(3), state(4));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    // Run the update step
    kf.updateStep(accel, lidar);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "kalman_filter");
    ros::NodeHandle nh;

    VectorXd state = VectorXd::Zero(6);
    state(5) = 1;
    MatrixXd cov = MatrixXd::Identity(6,6);
    
    kf.setState(state);
    kf.setCovariance(cov);
    kf.setGyro(0.01, 0); // Noise std, bias

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub(nh, "/scan", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, lidar_sub);
    sync.registerCallback(boost::bind(&KalmanCallback, _1, _2));

    ros::spin();
 }