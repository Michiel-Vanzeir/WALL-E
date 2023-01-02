#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <eigen3/Eigen/Dense>

struct GyroMeasurement{
    double yaw;
};

struct AccelMeasurement{
    double acceleration;
};

struct QuaternionMeasurement{
    float w;
    float x;
    float y;
    float z;
};

struct LidarMeasurement {
    std::vector<float> ranges;
};

struct EncoderMeasurement {
    float left_distance;
    float right_distance;
};

#endif // SENSORS_H