#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Dense>
#include "measurements.h"

// Include message definitions
#include <sensor_msgs/Imu.h>
#include <gabby_msgs/EncoderData.h>
#include <sensor_msgs/LaserScan.h>

using namespace Eigen;

class KalmanFilter {
    public:
        KalmanFilter();
        KalmanFilter(Vector4d init_state, Matrix4d init_cov, double yaw_std, double accel_std, double lidar_std);
        bool isInitialised() const {return initialized_;} 

        Vector4d getState() const {return state_;} 
        Matrix4d getCovariance() const {return covariance_;} 
        void setState(const Vector4d state) {state_ = state; initialized_ = true;}
        void setCovariance(const Matrix4d cov) {covariance_ = cov;} 

    
        void setAccelSTD(const double accel_std) {accel_noise_std = accel_std;}
        void setEncoderSTD(const double encoder_std) {encoder_noise_std = encoder_std;}
        void setLidarSTD(const double lidar_std) {lidar_noise_std = lidar_std;} 
        void setYawSTD(const double yaw_std) {yaw_noise_std = yaw_std;}

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void encoderCallback(const gabby_msgs::EncoderData::ConstPtr& msg);
        void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

        void predictionStep(double dt);
        void updateStep();

    private:
        bool initialized_;
        Vector4d state_;
        Matrix4d covariance_;

        double yaw_noise_std;
        double accel_noise_std;
        double encoder_noise_std;
        double lidar_noise_std;

        // Declare variables for storing measurements
        GyroMeasurement gyro_buffer;
        AccelMeasurement accel_buffer;
        QuaternionMeasurement orientation_buffer;
        EncoderMeasurement encoder_buffer;
        LidarMeasurement lidar_buffer;
};

#endif
