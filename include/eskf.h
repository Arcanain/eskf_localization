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
        //void Init(const GPS_Data& gps_data, xEst& xEst);
        void Init();
};

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

    //x.PEst.block<0, 0>(0, 0) = position_noise * Eigen::Matrix3d::Identity();
    //x.PEst.block<3, 3>(3, 3) = velocity_noise * Eigen::Matrix3d::Identity();
    //x.PEst.block<6, 6>(6, 6) = posture_noise * Eigen::Matrix3d::Identity();

    /*************/
    /* test code */
    /*************/
    xEst x;
    x.PEst.block<3, 3>(0, 0) = position_noise * Eigen::Matrix3d::Identity();
    x.PEst.block<3, 3>(3, 3) = velocity_noise * Eigen::Matrix3d::Identity();
    x.PEst.block<3, 3>(6, 6) = posture_noise * Eigen::Matrix3d::Identity();
    //x.PEst.block<3, 3>(0, 0) = position_noise * Eigen::MatrixXd::Identity(3, 3);

    //cout << Eigen::Matrix3d::Identity() << endl;
    cout << x.PEst.block<3, 3>(0, 0) << endl;
}

#endif // ESKF