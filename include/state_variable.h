#ifndef __STATE_VARIABLE__
#define __STATE_VARIABLE__

#include <iostream>
#include <Eigen/Core>

struct IMU_Data
{
    double timestamp;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct GPS_Data
{
    double timestamp;

    Eigen::Vector3d lla;
    Eigen::Vector3d ned;
};

struct xEst
{
    double timestamp;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d gravity;
    Eigen::Matrix<double, 18, 18> PEst;
    Eigen::Matrix<double, 18, 1> Error;
};

#endif // STATE_VARIABLE