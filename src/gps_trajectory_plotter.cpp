#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "state_variable.h"

using namespace std;

class GPS_Trajectory_Plotter
{
    private:
        ros::NodeHandle nh;

        // publisher subscriber
        ros::Publisher gps_pose_pub;
        ros::Publisher gps_path_pub;
        ros::Subscriber gps_sub;
        
        // gps variable
        GPS_Data gps_data;
        nav_msgs::Path gps_path;
        map_projection_reference map_ref;

        // for utbm_robocar_dataset_20190131_noimage.bag
        // Init gps position(latitude, longitude)
        double lat0 = 47.5115140833;
        double lon0 = 6.79310693333;
    public:
        // init
        GPS_Trajectory_Plotter();
        ~GPS_Trajectory_Plotter();

        // gps callback
        void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_msg);

        // gps pose calcurate
        int map_projection_init(struct map_projection_reference *ref, double lat_0, double lon_0);
        int map_projection_init_timestamped(struct map_projection_reference *ref, double lat_0, double lon_0);
        bool map_projection_initialized(const struct map_projection_reference *ref);
        int map_projection_project(const struct map_projection_reference *ref, double lat, double lon, float *x, float *y);
        double constrain(double val, double min, double max);
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

    map_projection_init(&map_ref, lat0, lon0);
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
    point.pose.orientation.z = 0;
    gps_path.poses.push_back(point);
    gps_pose_pub.publish(point);
    gps_path_pub.publish(gps_path);
}

int GPS_Trajectory_Plotter::map_projection_init(struct map_projection_reference *ref, double lat_0, double lon_0)
{
	return map_projection_init_timestamped(ref, lat_0, lon_0);
}

int GPS_Trajectory_Plotter::map_projection_init_timestamped(struct map_projection_reference *ref, double lat_0, double lon_0)
{
    ref->lat_rad = lat_0 * (M_PI / 180.0);
	ref->lon_rad = lon_0 * (M_PI / 180.0);
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);
	ref->init_done = true;

	return 0;
}

bool GPS_Trajectory_Plotter::map_projection_initialized(const struct map_projection_reference *ref)
{
	return ref->init_done;
}

int GPS_Trajectory_Plotter::map_projection_project(const struct map_projection_reference *ref, double lat, double lon, float *x, float *y)
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

double GPS_Trajectory_Plotter::constrain(double val, double min, double max)
{
    if (val > max) {
        return max;
    } else if (val < min) {
        return min;
    } else {
        return val;
    }
}

void GPS_Trajectory_Plotter::data_conversion_gps(const sensor_msgs::NavSatFixConstPtr& gps_msg, GPS_Data& gps_data)
{
    gps_data.timestamp = gps_msg->header.stamp.toSec();

    gps_data.lla = Eigen::Vector3d(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

    float x, y;
    map_projection_project(&map_ref, gps_msg->latitude, gps_msg->longitude, &x, &y);
    gps_data.ned = Eigen::Vector3d(x, y, -gps_msg->altitude);
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