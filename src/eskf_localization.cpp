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
    /*double lat0 = 47.5115140833;
    double lon0 = 6.79310693333;*/

    // hirayama 
    double lat0 = 35.056373;
    double lon0 = 137.171547;

    ROS_Interface ros_interface(n, lat0, lon0);

	while(ros::ok()){
        ros::spinOnce();
	}

    return 0;
}