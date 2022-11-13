#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "state_variable.h"
#include "geography.h"

using namespace std;

class GPS_Trajectory_Plotter
{
    private:
        ros::NodeHandle nh;

        // Publisher
        ros::Publisher gps_pose_pub;
        ros::Publisher gps_path_pub;

        // Subscriber
        ros::Subscriber gps_sub;
        
        // tf publish
        tf::TransformBroadcaster odom_to_baselink_broadcaster;
        geometry_msgs::TransformStamped odom_to_baselink_trans;

        // gps variable
        GPS_Data gps_data;
        nav_msgs::Path gps_path;
        map_projection_reference map_ref;

        // for utbm_robocar_dataset_20190131_noimage.bag
        // Init gps position(latitude, longitude)
        //double lat0 = 47.5115140833;
        //double lon0 = 6.79310693333;

        // gnss_log_data1
        double lat0 = 36.08295;
        double lon0 = 140.07705;
        double alt0 = 0.0;

        // gnss_log_data2
        //double lat0 = 36.08296;
        //double lon0 = 140.07699;
        //double alt0 = 0.0;

        // GEOGRAPHY Instance
        GEOGRAPHY geography;
    public:
        // init
        GPS_Trajectory_Plotter();
        ~GPS_Trajectory_Plotter();

        // gps callback
        void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg);

        // gps pose calcurate
        void data_conversion_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data);
};

GPS_Trajectory_Plotter::GPS_Trajectory_Plotter()
{
    std::cout << "Start GPS Trajectory Plotter!" << std::endl;

    gps_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gps_pose", 10);
    gps_path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 10);
    gps_sub = nh.subscribe("/fix", 10, &GPS_Trajectory_Plotter::gps_callback, this);

    gps_path.header.frame_id = "map";
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.seq = 0;

    geography.map_projection_init(&map_ref, lat0, lon0);
}

GPS_Trajectory_Plotter::~GPS_Trajectory_Plotter()
{
    std::cout << "Stop GPS Trajectory Plotter!" << std::endl;
}

void GPS_Trajectory_Plotter::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
    data_conversion_gps(gps_msg, gps_data);

    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.pose.position.x = gps_data.ned[0];
    point.pose.position.y = gps_data.ned[1];
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 0;
    point.pose.orientation.x = 0;
    point.pose.orientation.y = 0;
    point.pose.orientation.z = 1.0;
    gps_path.poses.push_back(point);
    gps_pose_pub.publish(point);
    gps_path_pub.publish(gps_path);

    // /odom to /base_link transform broadcast
    odom_to_baselink_trans.header.stamp = ros::Time::now();
    odom_to_baselink_trans.header.frame_id = "odom";
    odom_to_baselink_trans.child_frame_id = "base_link";
    odom_to_baselink_trans.transform.translation.x = gps_data.ned[0];
    odom_to_baselink_trans.transform.translation.y = gps_data.ned[1];
    odom_to_baselink_trans.transform.translation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.x = 0.0;
    odom_to_baselink_trans.transform.rotation.y = 0.0;
    odom_to_baselink_trans.transform.rotation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.w = 1.0;
    odom_to_baselink_broadcaster.sendTransform(odom_to_baselink_trans);
}

void GPS_Trajectory_Plotter::data_conversion_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data)
{
    gps_data.timestamp = gps_msg->header.stamp.toSec();

    gps_data.lla = Eigen::Vector3d(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

    /*
    float x, y;
    geography.map_projection_project(&map_ref, gps_msg->latitude, gps_msg->longitude, &x, &y);
    gps_data.ned = Eigen::Vector3d(x, y, -gps_msg->altitude);
    */
    
    //transform gps latitude and longitude coordinate to position in enu frame(Compute ECEF of NED origin)
    double enu[3];
    double lla[3] = {gps_msg->latitude, gps_msg->longitude, gps_msg->altitude};
    double ref[3] = {lat0, lon0, alt0};
    geography.lla2enu(enu, lla, ref);
    gps_data.ned = Eigen::Vector3d(enu[0], enu[1], enu[2]);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "gps_trajectory_plotter");

    GPS_Trajectory_Plotter gps_trajectory_plotter;
	while(ros::ok()){
        ros::spinOnce();
	}

    return 0;
}