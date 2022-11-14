#include "kalmanfilter.h"
#include "sensors.h"

double wrapAngle(double angle) {
    angle = fmod(angle, (2.0*M_PI));
    if (angle <= -M_PI){angle += (2.0*M_PI);}
    else if (angle > M_PI){angle -= (2.0*M_PI);}
    return angle;
}

KalmanFilter::KalmanFilter() {
    fl_initialised = false;
}

KalmanFilter::KalmanFilter(VectorXd state, MatrixXd covariance, VectorXd gyro_noise_std, VectorXd acccel_noise_std, double lidar_std) {
    fl_gyro_noise_std = gyro_noise_std;
    fl_accel_noise_std = acccel_noise_std;
    fl_lidar_noise_std = lidar_std;
    
    setState(state);
    setCovariance(covariance);

    fl_initialised = true;
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt) {
    if (!isInitialised()) {
        return;
    }
    VectorXd state = getState();
    MatrixXd cov = getCovariance();
    double pX = state(0);
    double pY = state(1);
    double pZ = state(2);
    double psiY = state(3);
    double psiZ = state(4);
    double velX = state(5);


    VectorXd F = VectorXd::Zero(6);
    F(0) = velX*cos(psiZ);
    F(1) = velX*sin(psiZ);
    F(2) = velX*sin(psiY);
    F(3) = gyro.psiZ;
    F(4) = gyro.psiY;
    F(5) = 0;

    MatrixXd J = MatrixXd::Zero(6,6); // Jacobean matrix
    J << 1, 0, 0, 0, -dt*velX*sin(psiZ), dt*cos(psiZ),
         0, 1, 0, 0, dt*velX*cos(psiZ), dt*sin(psiZ),
         0, 0, 1, dt*velX*cos(psiY), 0, dt*sin(psiY),
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    MatrixXd Q = MatrixXd::Zero(6,6); // Process noise covariance matrix
    Q(3,3) = dt*dt * fl_gyro_noise_std(1)*fl_gyro_noise_std(1);
    Q(4,4) = dt*dt * fl_gyro_noise_std(2)*fl_gyro_noise_std(2);
    Q(5,5) = dt*dt * fl_accel_noise_std(0)*fl_accel_noise_std(0);

    VectorXd state_pred = state + dt*F;
    state_pred(3) = wrapAngle(state_pred(3));
    state_pred(4) = wrapAngle(state_pred(4));
    MatrixXd cov_pred = J*cov*J.transpose() + Q;

    setState(state_pred);
    setCovariance(cov_pred);
}

void KalmanFilter::updateStep(AccelerometerMeasurement accel, LidarMeasurement lidar) {
    int x = 5;
}
