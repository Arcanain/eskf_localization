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
        ros::Publisher nav_odom_pub;

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
        map_projection_reference_s map_ref;
        
        /*
        // gps reference latitude and longitute
        double ref_lati;
        double ref_long;
        double ref_alti;

        // ned
        double kSemimajorAxis = 6378137;
        double kSemiminorAxis = 6356752.3142;
        double kFirstEccentricitySquared = 6.69437999014 * 0.001;
        double kSecondEccentricitySquared = 6.73949674228 * 0.001;
        double kFlattening = 1 / 298.257223563;
        */
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

        // gps pose calcurate{convert from (Lat,Long,Alt) to (North,East,Down)}
        int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0);
        int map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0);
        bool map_projection_initialized(const struct map_projection_reference_s *ref);
        int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x, float *y);
        double constrain(double val, double min, double max);

        /*
        // wgsconversion
        bool lla2enu(double enu[3], const double lla[3], const double ref_lla[3]);
        bool lla2xyz(double xyz[3], const double lla[3]);
        bool xyz2enu(double enu[3], const double xyz[3], const double ref_lla[3]);
        void rot3d(double R[3][3], const double reflat, const double reflon);
        void matrixMultiply(double C[3][3], const double A[3][3], const double B[3][3]);
        void matrixMultiply(double c[3], const double A[3][3], const double b[3]);
        void rot(double R[3][3], const double angle, const int axis);

        // ned conversion
        double initial_ecef_x;
        double initial_ecef_y;
        double initial_ecef_z;
        double north;
        double east;
        double down;

        Eigen::Matrix3d ecef_to_ned_matrix;

        double deg2Rad(const double degrees);
        void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x, double* y, double* z);
        Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians);
        void ecef2Ned(const double x, const double y, const double z, double* north, double* east, double* down);
        */
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
    estimated_pose_pub = nh.advertise<nav_msgs::Odometry>("/estimated_pose", 10);
    nav_odom_pub = nh.advertise<nav_msgs::Odometry>("/nav_odom", 10);

    // Subscriber for gazebo simulator
    //gps_sub = nh.subscribe("/gps/fix", 10, &ROS_Interface::gps_callback, this);
    gps_sub = nh.subscribe("/fix", 10, &ROS_Interface::gps_callback, this);
    imu_sub = nh.subscribe("/imu/data", 10, &ROS_Interface::imu_callback, this);

    gps_path.header.frame_id = "map";
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.seq = 0;

    estimated_path.header.frame_id = "map";
    estimated_path.header.stamp = ros::Time::now();
    estimated_path.header.seq = 0;

    estimated_pose.header.frame_id = "map";
    estimated_pose.child_frame_id = "base_link";
    estimated_pose.header.stamp = ros::Time::now();

    x.position = Eigen::Vector3d::Zero();
    x.velocity = Eigen::Vector3d::Zero();
    x.quaternion = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    x.acc_bias = Eigen::Vector3d::Zero();
    x.gyro_bias = Eigen::Vector3d::Zero();
    x.gravity = Eigen::Vector3d(0., 0., 9.81007); // ned frame
    //x.gravity = Eigen::Vector3d(0., 0., -9.81007); // enu frame
    x.PPred = Eigen::Matrix<double, 18, 18>::Zero();
    x.PEst = Eigen::Matrix<double, 18, 18>::Zero();
    x.cov = Eigen::Matrix<double, 18, 18>::Zero();
    x.error = Eigen::Matrix<double, 18, 1>::Zero();

    map_projection_init(&map_ref, lat, lon);
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
    point.pose.orientation.z = 0;
    gps_path.poses.push_back(point);
    gps_path_pub.publish(gps_path);

    // publish estimated_pose
    estimated_pose.pose.pose.position.x = x.position[0];
    estimated_pose.pose.pose.position.y = x.position[1];
    estimated_pose.pose.pose.position.z = 0.0;
    estimated_pose.pose.pose.orientation.w = x.quaternion.w();
    estimated_pose.pose.pose.orientation.x = x.quaternion.x();
    estimated_pose.pose.pose.orientation.y = x.quaternion.y();
    estimated_pose.pose.pose.orientation.z = x.quaternion.z();
    estimated_pose_pub.publish(estimated_pose);

    // /odom to /base_link transform broadcast
    odom_to_baselink_trans.header.stamp = ros::Time::now();
    odom_to_baselink_trans.header.frame_id = "odom";
    odom_to_baselink_trans.child_frame_id = "base_link";
    odom_to_baselink_trans.transform.translation.x = x.position[0];
    odom_to_baselink_trans.transform.translation.y = x.position[1];
    odom_to_baselink_trans.transform.translation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.w = 1.0;
    odom_to_baselink_trans.transform.rotation.x = 0.0;
    odom_to_baselink_trans.transform.rotation.y = 0.0;
    odom_to_baselink_trans.transform.rotation.z = 0.0;
    odom_to_baselink_broadcaster.sendTransform(odom_to_baselink_trans);

    // publish estimated_path
    geometry_msgs::PoseStamped estimated_point;
    estimated_point.header.frame_id = "map";
    estimated_point.header.stamp = ros::Time::now();
    estimated_point.pose.position.x = x.position[0];
    estimated_point.pose.position.y = x.position[1];
    estimated_point.pose.position.z = 0.0;
    estimated_point.pose.orientation.w = x.quaternion.w();
    estimated_point.pose.orientation.x = x.quaternion.x();
    estimated_point.pose.orientation.y = x.quaternion.y();
    estimated_point.pose.orientation.z = x.quaternion.z();

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

    float x, y;
    map_projection_project(&map_ref, gps_msg->latitude, gps_msg->longitude, &x, &y);
    gps_data.ned = Eigen::Vector3d(x, y, -gps_msg->altitude);

    /* transform gps latitude and longitude coordinate to position in enu frame(Compute ECEF of NED origin)
    double enu[3];
    double lla[3] = {gps_msg->latitude, gps_msg->longitude, gps_msg->altitude};
    double ref[3] = {ref_lati, ref_long, ref_alti};
    lla2enu(enu, lla, ref);

    double ecef_x;
    double ecef_y;
    double ecef_z;
    geodetic2Ecef(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, &ecef_x, &ecef_y, &ecef_z);
    ecef2Ned(ecef_x, ecef_y, ecef_z, &north, &east, &down);
    gps_data.ned = Eigen::Vector3d(north,east,down);
    
    nav_msgs::Odometry nav_odom;
    nav_odom.header.stamp = ros::Time::now();
    nav_odom.pose.pose.position.x = gps_msg->latitude;
    nav_odom.pose.pose.position.y = gps_msg->longitude;
    nav_odom.pose.pose.position.z = gps_msg->altitude;
    nav_odom.pose.pose.orientation.x = 0.0f;
    nav_odom.pose.pose.orientation.y = 0.0f;
    nav_odom.pose.pose.orientation.z = 0.0f;
    nav_odom.pose.pose.orientation.w = 1.0f;

    nav_odom_pub.publish(nav_odom);
    */
}

/***********************************************************************
 * convert from (Lat,Long,Alt) to (North,East,Down)
 **********************************************************************/
int ROS_Interface::map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0)
{
	return map_projection_init_timestamped(ref, lat_0, lon_0);
}

int ROS_Interface::map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0)
{
    ref->lat_rad = lat_0 * (M_PI / 180.0);
	ref->lon_rad = lon_0 * (M_PI / 180.0);
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);
	ref->init_done = true;

	return 0;
}

bool ROS_Interface::map_projection_initialized(const struct map_projection_reference_s *ref)
{
	return ref->init_done;
}

int ROS_Interface::map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x, float *y)
{
    static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000; //[m]

	if (!map_projection_initialized(ref)) {
		return -1;
	}

    const double lat_rad = lat * (M_PI / 180.0);
	const double lon_rad = lon * (M_PI / 180.0);

	const double sin_lat = sin(lat_rad);
	const double cos_lat = cos(lat_rad);

	const double cos_d_lon = cos(lon_rad - ref->lon_rad);

	const double arg = constrain(ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
	const double c = acos(arg);

	double k = 1.0;

	if (fabs(c) > 0) {
		k = (c / sin(c));
	}

	*x = static_cast<float>(k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
	*y = static_cast<float>(k * cos_lat * sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH);

	return 0;
}

double ROS_Interface::constrain(double val, double min, double max)
{
    if (val > max) {
        return max;
    } else if (val < min) {
        return min;
    } else {
        return val;
    }
}

/*
//------------------------------------------------------------------------------------------------
// WgsConversions::lla2enu [Public]  --- convert from (Lat,Long,Alt) to (East,North,Up)
//------------------------------------------------------------------------------------------------
bool ROS_Interface::lla2enu(double enu[3], const double lla[3], const double ref_lla[3])
{

    double xyz[3];

    if (!lla2xyz(xyz, lla))
        return 0;

    if (!xyz2enu(enu, xyz, ref_lla))
        return 0;
    
    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::lla2xyz [Public]  --- convert from (Lat,Long,Alt) to (ECEF X, ECEF Y, ECEF Z)
//------------------------------------------------------------------------------------------------
bool ROS_Interface::lla2xyz(double xyz[3], const double lla[3])
{

    if ((lla[0] < -90.0) | (lla[0] > +90.0) | (lla[1] < -180.0) | (lla[1] > +360.0))
    {
        std::cout << "WGS lat or WGS lon out of range" << std::endl;
        return 0;
    }

    double A_EARTH = 6378137.0;
    double flattening = 1.0 / 298.257223563;
    double NAV_E2 = (2.0 - flattening) * flattening; // also e^2
    double deg2rad = M_PI / 180.0;

    double slat = sin(lla[0] * deg2rad);
    double clat = cos(lla[0] * deg2rad);
    double r_n = A_EARTH / sqrt(1.0 - NAV_E2 * slat * slat);
    xyz[0] = (r_n + lla[2]) * clat * cos(lla[1] * deg2rad);
    xyz[1] = (r_n + lla[2]) * clat * sin(lla[1] * deg2rad);
    xyz[2] = (r_n * (1.0 - NAV_E2) + lla[2]) * slat;

    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::xyz2enu [Public]  --- convert from (ECEF X, ECEF Y, ECEF Z) to (East,North,Up)
//------------------------------------------------------------------------------------------------
bool ROS_Interface::xyz2enu(double enu[3], const double xyz[3], const double ref_lla[3])
{

    double ref_xyz[3], diff_xyz[3], R[3][3];

    // First, calculate the xyz of reflat, reflon, refalt
    if (!lla2xyz(ref_xyz, ref_lla))
        return 0;

    //Difference xyz from reference point
    diff_xyz[0] = xyz[0] - ref_xyz[0];
    diff_xyz[1] = xyz[1] - ref_xyz[1];
    diff_xyz[2] = xyz[2] - ref_xyz[2];

    rot3d(R, ref_lla[0], ref_lla[1]);

    matrixMultiply(enu, R, diff_xyz);

    return 1;
}

//--------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz [Private]  --- return the 3D rotation matrix to/from ECEF/ENU frame
//--------------------------------------------------------------------------------------------
void ROS_Interface::rot3d(double R[3][3], const double reflat, const double reflon)
{

    double R1[3][3], R2[3][3];

    rot(R1, 90 + reflon, 3);
    rot(R2, 90 - reflat, 1);

    matrixMultiply(R, R2, R1);
}

//------------------------------------------------------------------------------------------------
// WgsConversions::matrixMultiply [Private]  --- Multiply 3x3 matrix times another 3x3 matrix C=AB
//------------------------------------------------------------------------------------------------
void ROS_Interface::matrixMultiply(double C[3][3], const double A[3][3], const double B[3][3])
{

    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    C[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    C[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
    C[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
    C[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
    C[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];
}

//------------------------------------------------------------------------------------------------
// WgsConversions::matrixMultiply [Private]  --- Multiply 3x3 matrix times a 3x1 vector c=Ab
//------------------------------------------------------------------------------------------------
void ROS_Interface::matrixMultiply(double c[3], const double A[3][3], const double b[3])
{

    c[0] = A[0][0] * b[0] + A[0][1] * b[1] + A[0][2] * b[2];
    c[1] = A[1][0] * b[0] + A[1][1] * b[1] + A[1][2] * b[2];
    c[2] = A[2][0] * b[0] + A[2][1] * b[1] + A[2][2] * b[2];
}

//------------------------------------------------------------------------------------------------
// WgsConversions::rot [Private]  --- rotation matrix
//------------------------------------------------------------------------------------------------
void ROS_Interface::rot(double R[3][3], const double angle, const int axis)
{

    double cang = cos(angle * M_PI / 180);
    double sang = sin(angle * M_PI / 180);

    if (axis == 1)
    {
        R[0][0] = 1;
        R[0][1] = 0;
        R[0][2] = 0;
        R[1][0] = 0;
        R[2][0] = 0;
        R[1][1] = cang;
        R[2][2] = cang;
        R[1][2] = sang;
        R[2][1] = -sang;
    }
    else if (axis == 2)
    {
        R[0][1] = 0;
        R[1][0] = 0;
        R[1][1] = 1;
        R[1][2] = 0;
        R[2][1] = 0;
        R[0][0] = cang;
        R[2][2] = cang;
        R[0][2] = -sang;
        R[2][0] = sang;
    }
    else if (axis == 3)
    {
        R[2][0] = 0;
        R[2][1] = 0;
        R[2][2] = 1;
        R[0][2] = 0;
        R[1][2] = 0;
        R[0][0] = cang;
        R[1][1] = cang;
        R[1][0] = -sang;
        R[0][1] = sang;
    }
}


double ROS_Interface::deg2Rad(const double degrees)
{
    return (degrees / 180.0) * M_PI;
}

void ROS_Interface::geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x, double* y, double* z)
{
    // Convert geodetic coordinates to ECEF.
    // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
    double lat_rad = deg2Rad(latitude);
    double lon_rad = deg2Rad(longitude);
    double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
    *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
    *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
    *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
}

Eigen::Matrix3d ROS_Interface::nRe(const double lat_radians, const double lon_radians)
{
    const double sLat = sin(lat_radians);
    const double sLon = sin(lon_radians);
    const double cLat = cos(lat_radians);
    const double cLon = cos(lon_radians);
    
    Eigen::Matrix3d ret;
    ret(0, 0) = -sLat * cLon;
    ret(0, 1) = -sLat * sLon;
    ret(0, 2) = cLat;
    ret(1, 0) = -sLon;
    ret(1, 1) = cLon;
    ret(1, 2) = 0.0;
    ret(2, 0) = cLat * cLon;
    ret(2, 1) = cLat * sLon;
    ret(2, 2) = sLat;
    
    return ret;
}

void ROS_Interface::ecef2Ned(const double x, const double y, const double z, double* north, double* east, double* down)
{
    // Converts ECEF coordinate position into local-tangent-plane NED.
    // Coordinates relative to given ECEF coordinate frame.
    
    Eigen::Vector3d vect, ret;
    vect(0) = x - initial_ecef_x;
    vect(1) = y - initial_ecef_y;
    vect(2) = z - initial_ecef_z;
    ret = ecef_to_ned_matrix * vect;
    *north = ret(0);
    *east = ret(1);
    *down = -ret(2);
}
*/

#endif // ROS_INTERFACE