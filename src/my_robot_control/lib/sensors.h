#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <eigen3/Eigen/Dense>

struct GyroMeasurement{
    float psiX;
    float psiY;
    float psiZ;
};

struct AccelerometerMeasurement{
    float accelX;
    float accelY;
    float accelZ;
};

struct LidarMeasurement {
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    std::vector<float> ranges;
};



#endif // SENSORS_H