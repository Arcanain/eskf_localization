#ifndef __ROS_INTERFACE__
#define __ROS_INTERFACE__

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "eskf.h"
#include "state_variable.h"

using namespace std;

class ROS_Interface
{
    private:
        ros::NodeHandle nh;
        bool init;

        // Publisher
        ros::Publisher gps_path_pub;
        ros::Publisher estimated_path_pub;
        ros::Publisher estimated_pub;

        // Subscriber
        ros::Subscriber gps_sub;
        ros::Subscriber imu_sub;

        nav_msgs::Path gps_path;
        nav_msgs::Path estimated_path;
        geometry_msgs::Pose estimated_pose;

        State x;
        IMU_Data imu_data;
        GPS_Data gps_data;
        map_projection_reference_s map_ref;

        // ESKF
        ESKF eskf;
    public:
        ROS_Interface(ros::NodeHandle &n, double lat, double lon);
        ~ROS_Interface();

        // callback function
        void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
        void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);
        void data_conversion_imu(const sensor_msgs::ImuConstPtr& imu_msg, IMU_Data& imu_data);
        void data_conversion_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data);

        int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0);
        int map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0);
};

/***********************************************************************
 * Initialize 
 **********************************************************************/
ROS_Interface::ROS_Interface(ros::NodeHandle &n, double lat, double lon)
{
    cout << "ROS_Interface Start!" << endl;
    nh = n;
    init = false;

    // Publisher
    gps_path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 10);
    estimated_path_pub = nh.advertise<nav_msgs::Path>("/estimated_path", 10);
    estimated_pub = nh.advertise<geometry_msgs::Pose>("/estimated_pose", 10);

    // Subscriber for gazebo simulator
    gps_sub = nh.subscribe("/gps/fix", 10, &ROS_Interface::gps_callback, this);
    imu_sub = nh.subscribe("/imu/data", 10, &ROS_Interface::imu_callback, this);

    gps_path.header.frame_id = "map";
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.seq = 0;

    estimated_path.header.frame_id = "map";
    estimated_path.header.stamp = ros::Time::now();
    estimated_path.header.seq = 0;

    x.position = Eigen::Vector3d::Zero();
    x.velocity = Eigen::Vector3d::Zero();
    x.quaternion = Eigen::Quaterniond(0., 0., 0., 0.);
    x.acc_bias = Eigen::Vector3d::Zero();
    x.gyro_bias = Eigen::Vector3d::Zero();
    x.gravity = Eigen::Vector3d(0., 0., 9.81007);
    x.PPred = Eigen::Matrix<double, 18, 18>::Zero();
    x.PEst = Eigen::Matrix<double, 18, 18>::Zero();
    x.error = Eigen::Matrix<double, 18, 1>::Zero();

    map_projection_init(&map_ref, lat, lon);
    /*
    cout << map_ref.lat_rad << endl;
    cout << map_ref.lon_rad << endl;
    */
}

ROS_Interface::~ROS_Interface()
{
    cout << "ROS_Interface Finish" << endl;
}

void ROS_Interface::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    //cout << "IMU callback" << endl;
    data_conversion_imu(imu_msg, imu_data);

    eskf.Predict(imu_data, x);

    estimated_pose.position.x = x.position[0];
    estimated_pose.position.y = x.position[1];
    estimated_pose.position.z = x.position[2];
    estimated_pose.orientation.w = x.quaternion.w();
    estimated_pose.orientation.x = x.quaternion.x();
    estimated_pose.orientation.y = x.quaternion.y();
    estimated_pose.orientation.z = x.quaternion.z();
    estimated_pub.publish(estimated_pose);

    geometry_msgs::PoseStamped estimated_point;
    estimated_point.header.frame_id = "map";
    estimated_point.header.stamp = ros::Time::now();
    estimated_point.pose.position.x = x.position[0];
    estimated_point.pose.position.y = x.position[1];
    estimated_point.pose.position.z = 0;
    estimated_point.pose.orientation.w = x.quaternion.w();
    estimated_point.pose.orientation.x = x.quaternion.x();
    estimated_point.pose.orientation.y = x.quaternion.y();
    estimated_point.pose.orientation.z = x.quaternion.z();

    estimated_path.poses.push_back(estimated_point);
    estimated_path_pub.publish(estimated_path);
}

void ROS_Interface::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
    //cout << "GPS callback" << endl;
    data_conversion_gps(gps_msg, gps_data);
}   

void ROS_Interface::data_conversion_imu(const sensor_msgs::ImuConstPtr& imu_msg, IMU_Data& imu_data)
{
    imu_data.timestamp = imu_msg->header.stamp.toSec();

    imu_data.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x, 
                                   imu_msg->linear_acceleration.y,
                                   imu_msg->linear_acceleration.z);

    imu_data.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                    imu_msg->angular_velocity.y,
                                    imu_msg->angular_velocity.z);
}

void ROS_Interface::data_conversion_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data)
{
    gps_data.timestamp = gps_msg->header.stamp.toSec();

    gps_data.lla = Eigen::Vector3d(gps_msg->latitude,
                                   gps_msg->longitude,
                                   gps_msg->altitude);
}

int ROS_Interface::map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0)
{
	return map_projection_init_timestamped(ref, lat_0, lon_0);
}

int ROS_Interface::map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0)
{

	//ref->lat_rad = radians(lat_0);
	//ref->lon_rad = radians(lon_0);
    ref->lat_rad = lat_0 * (M_PI / 180.0);
	ref->lon_rad = lon_0 * (M_PI / 180.0);
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);

	// ref->timestamp = timestamp;
	ref->init_done = true;

	return 0;
}


#endif // ROS_INTERFACE