#include <ros/ros.h>

#include "ros_interface.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "test_eskf_localization");
    ros::NodeHandle n;

    // gazebo simulator
    double lat0 = 49.89999999967053;
    double lon0 = 8.899999999175634;
    ROS_Interface ros_interface(n, lat0, lon0);

	while(ros::ok()){
        ros::spinOnce();
	}

    return 0;
}