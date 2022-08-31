#ifndef __ESKF__
#define __ESKF__

#include <iostream>
#include "state_variable.h"

using namespace std;

class ESKF
{
    private:
        // Init
        double position_noise = 1.2;
        double velocity_noise = 10.0;
        double posture_noise  = 1.0;

        // Predict
        double acc_noise = 5e-4;
        double gyro_noise = 5e-4;
        double acc_bias_noise = 5e-4;
        double gyro_bias_noise = 5e-4;

        Eigen::Matrix<double, 18, 18> Fx;
        Eigen::Matrix<double, 18, 12> Fi;
        Eigen::Matrix<double, 12, 12> Qi;

        // Correct
        double pose_noise = 1.2;

        Eigen::Matrix<double, 3, 18> H;

    public:
        ESKF();
        ~ESKF();
        //void Init(const GPS_Data& gps_data, State& x);
        void Init();
        //void Predict(const IMU_Data& imu_data, State& x);
        void Predict();
        //void Update(const GPS_Data& gps_data, State& x);
        void Correct();
        //void State_update(State& x);
        void State_update();
        //void ESKF::Error_State_Reset(State& x);
        void Error_State_Reset();

        // Quaternion
        Eigen::Quaterniond euler_to_quatertion(Eigen::Vector3d euler);
        Eigen::Quaterniond convert_euler_to_quatertion(const double roll, const double pitch, const double yaw);
        Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q);
        Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d& vec);

        // Predict
        Eigen::Matrix<double, 18, 18> calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Matrix3d R, const double dt);
        Eigen::Matrix<double, 18, 12> calcurate_Jacobian_Fi();
        Eigen::Matrix<double, 12, 12> calcurate_Jacobian_Qi(const double dt);

        // Correct
        Eigen::Matrix<double, 3, 18> calcurate_Jacobian_H(State& x);
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

//void ESKF::Init(const GPS_Data& gps_data, State& x)
void ESKF::Init()
{
    /*************/
    /* test code */
    /*************/
    State x;

    //x.timestamp = gps_data.timestamp;
    //x.position  = gps_data.ned;

    x.PPred.block<3, 3>(0, 0) = position_noise * Eigen::Matrix3d::Identity();
    x.PPred.block<3, 3>(3, 3) = velocity_noise * Eigen::Matrix3d::Identity();
    x.PPred.block<3, 3>(6, 6) = posture_noise * Eigen::Matrix3d::Identity();
    x.PEst.block<3, 3>(0, 0) = position_noise * Eigen::Matrix3d::Identity();
    x.PEst.block<3, 3>(3, 3) = velocity_noise * Eigen::Matrix3d::Identity();
    x.PEst.block<3, 3>(6, 6) = posture_noise * Eigen::Matrix3d::Identity();
}

// https://qiita.com/rsasaki0109/items/e969ad632cf321e25a6a
/***********************************************************************
 * ESKF Predict Step
 **********************************************************************/
/* 
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

//void Predict(const IMU_Data& imu_data, State& x);
void ESKF::Predict()
{
    /*************/
    /* test code */
    /*************/
    State x;

    IMU_Data imu_data;
    const double dt = imu_data.timestamp - x.timestamp;
    x.timestamp = imu_data.timestamp;

    Eigen::Matrix3d R = x.quaternion.toRotationMatrix();
    //cout << R << endl;
    
    // state update //
    x.position = x.position + x.velocity * dt + 0.5 * (R * (imu_data.acc - x.acc_bias) + x.gravity) * dt * dt;
    x.velocity = x.velocity + (R * (imu_data.acc - x.acc_bias) + x.gravity) * dt;
    x.quaternion = kronecker_product(x.quaternion, euler_to_quatertion((imu_data.gyro - x.gyro_bias) * dt));

    /*
    Eigen::Quaterniond quat_wdt = Eigen::Quaterniond(
    Eigen::AngleAxisd((imu_data.gyro(0) - x.gyro_bias(0)) * dt, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd((imu_data.gyro(1) - x.gyro_bias(1)) * dt, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd((imu_data.gyro(2) - x.gyro_bias(2)) * dt, Eigen::Vector3d::UnitZ()));
    x.quaternion = quat_wdt * x.quaternion;
    */

    // calcurate Jacobian Fx
    Fx = calcurate_Jacobian_Fx(imu_data.acc, x.acc_bias, R, dt);
    //cout << Fx << endl;

    // calcurate Jacobian Fi
    Fi = calcurate_Jacobian_Fi();
    //cout << Fi << endl;

    // ccalcurate Jacobian Qi
    Qi = calcurate_Jacobian_Qi(dt);
    //cout << Qi << endl;

    // calcurate PPred
    x.PPred = Fx * x.PEst * Fx.transpose() + Fi * Qi * Fi.transpose();
    //cout << x.PPred << endl;
}

Eigen::Matrix<double, 18, 18> ESKF::calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Matrix3d R, const double dt)
{
    Eigen::Matrix<double, 18, 18> Fx = Eigen::Matrix<double, 18, 18>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = - skewsym_matrix(R * (acc - acc_bias)) * dt;
    Fx.block<3, 3>(3, 9) = - R * dt;
    Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(6, 12) = - R * dt;

    return Fx;
}

Eigen::Matrix<double, 18, 12> ESKF::calcurate_Jacobian_Fi()
{
    Eigen::Matrix<double, 18, 12> Fi = Eigen::Matrix<double, 18, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    return Fi;
}

Eigen::Matrix<double, 12, 12> ESKF::calcurate_Jacobian_Qi(const double dt)
{
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = dt * dt * acc_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = dt * dt * gyro_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = dt * acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = dt * gyro_bias_noise * Eigen::Matrix3d::Identity();

    return Qi;
}

// https://qiita.com/rsasaki0109/items/e969ad632cf321e25a6a
/***********************************************************************
 * ESKF Correct Step
 **********************************************************************/
/*
* y = pobs = [xobs yobs zobs]
*
* K = P_k H^T (H P_k H^T + R)^{-1}
*
* dx = K (y_k - p_k )
*
* p_x = p_{k-1} + dp_k
* v_k = v_{k-1} + dv_k
* q_k = Rot(dth) q_{k-1}
*
* P_k = (I - KH)*P_{k-1}
*/
//void Correct(const GPS_Data&, State&)
void ESKF::Correct()
{
    /*************/
    /* test code */
    /*************/
    State x;
    GPS_Data gps_data;

    Eigen::Vector3d Y(gps_data.ned[0], gps_data.ned[1], gps_data.ned[2]);
    Eigen::Vector3d X(x.position[0], x.position[1], x.position[2]);
    Eigen::Matrix3d V = pose_noise * Eigen::Matrix3d::Identity();

    // calcurate Jacobian H
    H = calcurate_Jacobian_H(x);
    //cout << H << endl;

    Eigen::MatrixXd K = x.PPred * H.transpose() * (H * x.PPred * H.transpose() + V).inverse();
    x.error = K * (Y - X);
    //cout << x.error << endl;
    x.PEst = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * x.PPred;
    //cout << x.PEst << endl;
}

Eigen::Matrix<double, 3, 18> ESKF::calcurate_Jacobian_H(State& x)
{
    Eigen::Matrix<double, 3, 19> Hx = Eigen::Matrix<double, 3, 19>::Zero();
    Hx.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 19, 18> Xx = Eigen::Matrix<double, 19, 18>::Identity();
    Eigen::Matrix<double, 4, 3> Q_theta;
    Q_theta << - x.quaternion.x(), - x.quaternion.y(), - x.quaternion.z(), 
                x.quaternion.w(), x.quaternion.z(), - x.quaternion.y(),
                - x.quaternion.z(), x.quaternion.w(), x.quaternion.x(),
                x.quaternion.y(), - x.quaternion.x(), x.quaternion.w();
    Q_theta *= 0.5;
    Xx.block<4, 3>(6, 6) = Q_theta;
    
    Eigen::Matrix<double, 3, 18> H = Hx * Xx;

    return H;
}

/***********************************************************************
 * State Update
 **********************************************************************/
//void state_update(State& x)
void ESKF::State_update()
{
    /*************/
    /* test code */
    /*************/
    State x;

    Eigen::Vector3d error_pos = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(0, 0).transpose().data());
    Eigen::Vector3d error_vel = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(3, 0).transpose().data());
    Eigen::Vector3d error_ori = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(6, 0).transpose().data());
    Eigen::Quaterniond error_quat = euler_to_quatertion(error_ori);

    Eigen::Vector3d error_acc_bias = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(9, 0).transpose().data());
    Eigen::Vector3d error_gyr_bias = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(12, 0).transpose().data());
    Eigen::Vector3d error_gra = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(15, 0).transpose().data());

    x.position = x.position + error_pos;
    x.velocity = x.velocity + error_vel;
    x.quaternion = kronecker_product(error_quat, x.quaternion);
    x.acc_bias = x.acc_bias + error_acc_bias;
    x.gyro_bias = x.gyro_bias + error_gyr_bias;
    // x.gravity = x.gravity + error_gra;
}

//void ESKF::Error_State_Reset(State& x)
void ESKF::Error_State_Reset()
{
    /*************/
    /* test code */
    /*************/
    State x;

    x.error.setZero();
    //cout << x.error << endl;
}

/***********************************************************************
 * Quaternion
 **********************************************************************/
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


// https://qiita.com/take4eng/items/ae487c82a6f7d60ceba8
Eigen::Quaterniond ESKF::euler_to_quatertion(Eigen::Vector3d euler)
{
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    Eigen::Quaterniond quad = convert_euler_to_quatertion(roll, pitch, yaw);

    return quad;
}

Eigen::Quaterniond ESKF::convert_euler_to_quatertion(const double roll, const double pitch, const double yaw)
{
    double cr = cos(0.5 * roll);
    double sr = sin(0.5 * roll);
    double cp = cos(0.5 * pitch);
    double sp = sin(0.5 * pitch);
    double cy = cos(0.5 * yaw);
    double sy = sin(0.5 * yaw);

    Eigen::Vector4d vq;

    vq[0] = cy * cp * cr + sy * sp * sr;
    vq[1] = cy * cp * sr - sy * sp * cr;
    vq[2] = sy * cp * sr + cy * sp * cr;
    vq[3] = sy * cp * cr - cy * sp * sr;

    Eigen::Quaterniond q(vq);
    
    return q;
}

#endif // ESKF