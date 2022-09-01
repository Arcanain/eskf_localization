#include <ros/ros.h>
#include <iostream>

#include "eskf.h"
#include "ros_interface.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "eskf_localization");
	ros::NodeHandle nh;

	ESKF eskf;

    //ros::Rate loop_rate(50);
	while(ros::ok()){
		ros::spinOnce();
		/*
		eskf.Init();
		eskf.Predict();
		eskf.Correct();
		eskf.State_update();
		eskf.Error_State_Reset();
		*/
		//loop_rate.sleep();
	}

    return 0;
}