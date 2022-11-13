#ifndef __ROS_INTERFACE__
#define __ROS_INTERFACE__

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

#include "eskf.h"
#include "state_variable.h"
#include "geography.h"

using namespace std;

class ROS_Interface
{
    private:
        ros::NodeHandle nh;
        bool init;

        // Publisher
        ros::Publisher gps_path_pub;
        ros::Publisher estimated_path_pub;
        ros::Publisher estimated_pose_pub;

        // Subscriber
        ros::Subscriber gps_sub;
        ros::Subscriber imu_sub;
        
        // tf publish
        tf::TransformBroadcaster odom_to_baselink_broadcaster;
        geometry_msgs::TransformStamped odom_to_baselink_trans;
        
        // publish data
        nav_msgs::Path gps_path;
        nav_msgs::Path estimated_path;
        nav_msgs::Odometry estimated_pose;

        // ESKF variable
        State x;
        IMU_Data imu_data;
        GPS_Data gps_data;
        map_projection_reference map_ref;
        double lat0;
        double lon0;
        double alt0;
        
        // ESKF Instance
        ESKF eskf;

        // GEOGRAPHY Instance
        GEOGRAPHY geography;
    public:
        // Init
        ROS_Interface(ros::NodeHandle &n, double lat, double lon);
        ~ROS_Interface();

        // callback function
        void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
        void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);
        void data_conversion_imu(const sensor_msgs::ImuConstPtr& imu_msg, IMU_Data& imu_data);
        void data_conversion_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data);
};

/***********************************************************************
 * Initialize 
 **********************************************************************/
ROS_Interface::ROS_Interface(ros::NodeHandle &n, double lat, double lon)
{
    // init ROS
    cout << "ROS_Interface Start!" << endl;
    nh = n;
    init = false;

    // Publisher
    gps_path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 10);
    estimated_path_pub = nh.advertise<nav_msgs::Path>("/estimated_path", 10);
    estimated_pose_pub = nh.advertise<nav_msgs::Odometry>("/estimated_pose", 10);

    // Subscriber
    gps_sub = nh.subscribe("/fix", 10, &ROS_Interface::gps_callback, this);
    imu_sub = nh.subscribe("/imu/data", 10, &ROS_Interface::imu_callback, this);

    // init gps_path
    gps_path.header.frame_id = "map";
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.seq = 0;

    // init estimated_path
    estimated_path.header.frame_id = "map";
    estimated_path.header.stamp = ros::Time::now();
    estimated_path.header.seq = 0;

    // init estimated_pose
    estimated_pose.header.frame_id = "map";
    estimated_pose.child_frame_id = "base_link";
    estimated_pose.header.stamp = ros::Time::now();

    // init state
    /*
    x.position = Eigen::Vector3d::Zero();
    x.velocity = Eigen::Vector3d::Zero();
    */
    //x.quaternion = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    /*
    x.acc_bias = Eigen::Vector3d::Zero();
    x.gyro_bias = Eigen::Vector3d::Zero();
    */
    
    //x.gravity = Eigen::Vector3d(0., 0., 9.81007); // ned frame
    x.gravity = Eigen::Vector3d(0., 0., -9.81007); // enu frame
    /*
    x.PEst = Eigen::Matrix<double, 18, 18>::Zero();
    x.error = Eigen::Matrix<double, 18, 1>::Zero();
    */

    // init reference lat, lon projection
    geography.map_projection_init(&map_ref, lat, lon);
    lat0 = lat;
    lon0 = lon;
    alt0 = 0.0;
}

ROS_Interface::~ROS_Interface()
{
    cout << "ROS_Interface Finish" << endl;
}

/***********************************************************************
 * Callback function(imu, gps)
 **********************************************************************/
void ROS_Interface::imu_callback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    data_conversion_imu(imu_msg, imu_data);

    if (!init) {
        return;
    }

    eskf.Predict(imu_data, x);
}

void ROS_Interface::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
    data_conversion_gps(gps_msg, gps_data);

    if(!init){
        eskf.Init(gps_data, x);
        init = true;
        return;
    }

    eskf.Correct(gps_data, x);
    eskf.State_update(x);
    eskf.Error_State_Reset(x);

    // publish gps_path
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
    gps_path_pub.publish(gps_path);

    // publish estimated_pose
    estimated_pose.pose.pose.position.x = x.position[0];
    estimated_pose.pose.pose.position.y = x.position[1];
    estimated_pose.pose.pose.position.z = 0.0;
    estimated_pose.pose.pose.orientation.x = x.quaternion.x();
    estimated_pose.pose.pose.orientation.y = x.quaternion.y();
    estimated_pose.pose.pose.orientation.z = x.quaternion.z();
    estimated_pose.pose.pose.orientation.w = x.quaternion.w();
    estimated_pose_pub.publish(estimated_pose);

    // /odom to /base_link transform broadcast
    odom_to_baselink_trans.header.stamp = ros::Time::now();
    odom_to_baselink_trans.header.frame_id = "odom";
    odom_to_baselink_trans.child_frame_id = "base_link";
    odom_to_baselink_trans.transform.translation.x = x.position[0];
    odom_to_baselink_trans.transform.translation.y = x.position[1];
    odom_to_baselink_trans.transform.translation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.x = 0.0;
    odom_to_baselink_trans.transform.rotation.y = 0.0;
    odom_to_baselink_trans.transform.rotation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.w = 1.0;
    odom_to_baselink_broadcaster.sendTransform(odom_to_baselink_trans);

    // publish estimated_path
    geometry_msgs::PoseStamped estimated_point;
    estimated_point.header.frame_id = "map";
    estimated_point.header.stamp = ros::Time::now();
    estimated_point.pose.position.x = x.position[0];
    estimated_point.pose.position.y = x.position[1];
    estimated_point.pose.position.z = 0.0;
    estimated_point.pose.orientation.x = x.quaternion.x();
    estimated_point.pose.orientation.y = x.quaternion.y();
    estimated_point.pose.orientation.z = x.quaternion.z();
    estimated_point.pose.orientation.w = x.quaternion.w();

    estimated_path.poses.push_back(estimated_point);
    estimated_path_pub.publish(estimated_path);
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

    /*
    double ecef_x;
    double ecef_y;
    double ecef_z;
    geodetic2Ecef(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, &ecef_x, &ecef_y, &ecef_z);
    ecef2Ned(ecef_x, ecef_y, ecef_z, &north, &east, &down);
    gps_data.ned = Eigen::Vector3d(north,east,down);
    */
}

#endif // ROS_INTERFACE