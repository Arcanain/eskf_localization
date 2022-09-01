#ifndef __ROS_INTERFACE__
#define __ROS_INTERFACE__

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "state_variable.h"
#include "eskf.h"

using namespace std;

class ROS_Interface
{
    private:
        ros::NodeHandle nh;
        bool init;
    public:
    ROS_Interface(ros::NodeHandle &n, double lat, double lon);
    ~ROS_Interface();
};

/***********************************************************************
 * Initialize 
 **********************************************************************/
ROS_Interface::ROS_Interface(ros::NodeHandle &n, double lat, double lon)
{
    cout << "ROS_Interface Start!" << endl;
    nh = n;
    cout << lat << endl;
}

ROS_Interface::~ROS_Interface()
{
    cout << "ROS_Interface Finish" << endl;
}

#endif // ROS_INTERFACE