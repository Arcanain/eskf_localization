#include <ros/ros.h>
#include <iostream>

#include "ros_interface.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "test_eskf_localization");
    ros::NodeHandle n;

    double lat0 = 47.5115140833;
    double lon0 = 6.79310693333;
    ROS_Interface ros_interface(n, lat0, lon0);

	while(ros::ok()){
        ros::spinOnce();
	}

    return 0;
}