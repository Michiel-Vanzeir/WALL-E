#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#pragma once

#include <eigen3/Eigen/Dense>
#include "measurements.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter {
    public:
        KalmanFilter();
        KalmanFilter(VectorXd state, MatrixXd covariance, VectorXd gyro_noise_std, VectorXd acccel_noise_std, double lidar_std);
        bool isInitialised() const {return fl_initialised;} // 

        VectorXd getState() const {return fl_state;} //
        MatrixXd getCovariance() const {return fl_covariance;} //
        void setState(const VectorXd& state) {fl_state = state; fl_initialised = true;} //
        void setCovariance(const MatrixXd& cov) {fl_covariance = cov;} //
        void setGyro(const VectorXd gyro_noise_std) {fl_gyro_noise_std = gyro_noise_std;} //

        void predictionStep(GyroMeasurement gyro, double dt);
        void updateStep(AccelerometerMeasurement accel, LidarMeasurement lidar);

    private:
        bool fl_initialised = true;
        VectorXd fl_state;
        MatrixXd fl_covariance;

        VectorXd fl_accel_noise_std;
        VectorXd fl_gyro_noise_std;
        double fl_lidar_noise_std;
};

#endif
