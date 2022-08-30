#ifndef __ESKF__
#define __ESKF__

#include <iostream>
#include "state_variable.h"

using namespace std;

class ESKF
{
    private:
        double position_noise = 1.2;
        double velocity_noise = 10.0;
        double posture_noise  = 1.0;
    public:
        ESKF();
        ~ESKF();
        //void Init(const GPS_Data& gps_data, xEst& x);
        void Init();
        //void Predict(const IMU_Data& imu_data, xEst& x);
        void Predict();

        Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q);
        Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d& vec);
};

/***********************************************************************
 * Initialize 
 **********************************************************************/
ESKF::ESKF()
{
    cout << "ESKF Start!" << endl;
}

ESKF::~ESKF()
{
    cout << "ESKF Finish" << endl;
}

//void ESKF::Init(const GPS_Data& gps_data, xEst& x)
void ESKF::Init()
{
    //x.timestamp = gps_data.timestamp;
    //x.position  = gps_data.ned;

    /*************/
    /* test code */
    /*************/
    xEst x;
    x.PEst.block<3, 3>(0, 0) = position_noise * Eigen::Matrix3d::Identity();
    x.PEst.block<3, 3>(3, 3) = velocity_noise * Eigen::Matrix3d::Identity();
    x.PEst.block<3, 3>(6, 6) = posture_noise * Eigen::Matrix3d::Identity();

    //cout << x.PEst.block<3, 3>(0, 0) << endl;
}

/***********************************************************************
 * ESKF Predict Step
 **********************************************************************/
/* state
* x  = [p v q] = [x y z vx vy vz qx qy qz qw]
* dx = [dp dv dth] = [dx dy dz dvx dvy dvz dthx dthy dthz]
*
* pos_k = pos_{k-1} + vel_k * dt + (1/2) * (Rot(q_{k-1}) acc_{k-1}^{imu} - g) *dt^2
* vel_k = vel_{k-1} + (Rot(quat_{k-1})) acc_{k-1}^{imu} - g) *dt
* quat_k = Rot(w_{k-1}^{imu}*dt)*quat_{k-1}
*
* covariance
* P_{k} = F_k P_{k-1} F_k^T + L Q_k L^T
*/

//void Predict(const IMU_Data& imu_data, xEst& x);
void ESKF::Predict()
{
    /*************/
    /* test code */
    /*************/
    xEst x;
    IMU_Data imu_data;
    const double dt = imu_data.timestamp - x.timestamp;
    x.timestamp = imu_data.timestamp;

    Eigen::Matrix3d R = x.quaternion.toRotationMatrix();
    //cout << R << endl;
    
    // state update //
    x.position = x.position + x.velocity * dt + 0.5 * (R * (imu_data.acc - x.acc_bias) + x.gravity) * dt * dt;
    x.velocity = x.velocity + (R * (imu_data.acc - x.acc_bias) + x.gravity) * dt;
    //x.quaternion = kronecker_product(x.quaternion, euler_to_quatertion((imu_data.gyro - x.gyro_bias) * dt));

    Eigen::Quaterniond quat_wdt = Eigen::Quaterniond(
    Eigen::AngleAxisd((imu_data.gyro(0) - x.gyro_bias(0)) * dt, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd((imu_data.gyro(1) - x.gyro_bias(1)) * dt, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd((imu_data.gyro(2) - x.gyro_bias(2)) * dt, Eigen::Vector3d::UnitZ()));
    x.quaternion = quat_wdt * x.quaternion;

    // calcurate Fx
    
    // calcurate Fi

    // calcurate PPred
}

Eigen::Quaterniond ESKF::kronecker_product(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q)
{
    Eigen::Quaterniond res;
    res.w() = p.w() * q.w() - p.x() * q.x() - p.y() * q.y() - p.z() * q.z();
    res.x() = p.w() * q.x() + p.x() * q.w() + p.y() * q.z() - p.z() * q.y();
    res.y() = p.w() * q.y() - p.x() * q.z() + p.y() * q.w() + p.z() * q.x();
    res.z() = p.w() * q.z() + p.x() * q.y() - p.y() * q.x() + p.z() * q.w();
    return res;
}

Eigen::Matrix3d ESKF::skewsym_matrix(const Eigen::Vector3d& vec)
{
    Eigen::Matrix3d mat;
    mat <<  0.0,   -vec(2),  vec(1),
          vec(2),  0.0,   -vec(0),
         -vec(1),  vec(0),  0.0;
    return mat;
}

#endif // ESKF