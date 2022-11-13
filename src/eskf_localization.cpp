#include <ros/ros.h>

#include "ros_interface.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "eskf_localization");
    ros::NodeHandle n;

    // gazebo simulator
    /*double lat0 = 49.89999999967053;
    double lon0 = 8.899999999175634;*/
	
    // utbm_robocar_dataset_20190131_noimage.bag
    //double lat0 = 47.5115140833;
    //double lon0 = 6.79310693333;

    // hirayama 
    //double lat0 = 35.055033181671;
    //double lon0 = 137.17151228476;

    // tsukuba environment //double lat0 = 36.08324140969784;
    //double lon0 = 140.07696509173357;
    //double lat0 = 36.08324140969784;
    //double lon0 = 140.07696509173357;
    double lat0 = 36.08263197663968;
    double lon0 = 140.07660776463374;

    // tsukuba parking 1
    //double lat0 = 36.09183981305693;
    //double lon0 = 140.10840555631;

    // park
    //double lat0 = 35.05041567088393;
    //double lon0 = 137.18095104107948;

    ROS_Interface ros_interface(n, lat0, lon0);

	while(ros::ok()){
        ros::spinOnce();
	}

    return 0;
}