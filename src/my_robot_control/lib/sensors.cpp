#include "sensors.h"


// Gyro Sensor
GyroSensor::GyroSensor(double noise_std, double bias) {
    if (!imu.begin()) {
        return false; // NEED TO DO SOMETHING HERE
    }
    bno.setExtCrystalUse(true);
    m_noise_std = noise_std;
    m_bias = bias;
}

void GyroSensor::setGyroNoiseStd(double std){m_noise_std = std;}
void GyroSensor::setGyroBias(double bias){m_bias = bias;}

GyroMeasurement GyroSensor::generateGyroMeasurement(double sensor_yaw_rate)
{
    sensors_event_t event;
    imu.getEvent(&event);

    // event.gyro.x, event.gyro.y, event.gyro.z (in rad/s)
    GyroMeasurement meas;
    meas.x_dot = event.gyro.x + m_bias; // + m_noise_std;
    meas.y_dot = event.gyro.y + m_bias; //+ m_noise_std;
    meas.z_dot = event.gyro.z + m_bias; //+ m_noise_std;

    return meas;
}
