#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

#include <boost/date_time.hpp>

class Odometry_Publisher
{
    private:
        ros::NodeHandle nh;

        // Publisher
        ros::Publisher odom_pub;
        ros::Publisher odom_path_pub;

        // Subscriber
        ros::Subscriber linear_vel_sub;
        ros::Subscriber angular_vel_sub;
        ros::Subscriber estimated_pose_sub;

        // timer setting
        ros::Time current_time = ros::Time::now();
        ros::Time last_time = ros::Time::now();
        float dt = (current_time - last_time).toSec();

        // publish data
        geometry_msgs::Pose odom;
        nav_msgs::Path odom_path;
    public:
        // Init Odometry_Publisher
        Odometry_Publisher();
        ~Odometry_Publisher();

        // /vel callback
        void linear_vel_callback(const geometry_msgs::TwistStamped &linear_vel_msg);
        void angular_vel_callback(const geometry_msgs::TwistStamped &angular_vel_msg);
        float linear_x  = 0.0;  //[m/s]
        float linear_y  = 0.0;  //[m/s]
        float angular_z = 0.0;  //[rad/s]
        bool sub_linear_vel = false;
        bool sub_angular_vel = false;

        // estimated_pose callback
        void estimated_pose_callback(const nav_msgs::Odometry &estimated_pose_msg);
        bool init_estimated_pose = false;

        // odometry publish
        void update_odometry();
};

/***********************************************************************
 * Initialize 
 **********************************************************************/
Odometry_Publisher::Odometry_Publisher()
{
    // Publisher
    odom_pub = nh.advertise<geometry_msgs::Pose>("/odom", 50);
    odom_path_pub = nh.advertise<nav_msgs::Path>("/odom_path", 10);

    // Subscriber
    linear_vel_sub = nh.subscribe("/vel", 10, &Odometry_Publisher::linear_vel_callback, this);
    angular_vel_sub = nh.subscribe("/velocity", 10, &Odometry_Publisher::angular_vel_callback, this);
    estimated_pose_sub = nh.subscribe("/estimated_pose", 10, &Odometry_Publisher::estimated_pose_callback, this);

    // init odometry
    odom.position.x = 0.0;
    odom.position.y = 0.0;
    odom.position.z = 0.0;
    odom.orientation.x = 0.0;
    odom.orientation.y = 0.0;
    odom.orientation.z = 0.0;
    odom.orientation.w = 1.0;

    // init odometry path 
    odom_path.header.frame_id = "map";
    odom_path.header.stamp = ros::Time::now();
    odom_path.header.seq = 0;
}

Odometry_Publisher::~Odometry_Publisher()
{
    std::cout << "Finish Odom Publisher!" << std::endl;
}

/***********************************************************************
 * Callback function(/cmd_vel, /estimated_pose)
 **********************************************************************/
void Odometry_Publisher::linear_vel_callback(const geometry_msgs::TwistStamped &linear_vel_msg)
{
    //std::cout << "linear vel callback" << std::endl;
    linear_x  = linear_vel_msg.twist.linear.x;  //[m/s]
    linear_y  = linear_vel_msg.twist.linear.y;  //[m/s]
    sub_linear_vel = true;
}

void Odometry_Publisher::angular_vel_callback(const geometry_msgs::TwistStamped &angular_vel_msg)
{
    //std::cout << "angular vel callback" << std::endl;
    angular_z = angular_vel_msg.twist.angular.z;  //[rad/s]
    sub_angular_vel = true;
}

void Odometry_Publisher::estimated_pose_callback(const nav_msgs::Odometry &estimated_pose_msg)
{
    //std::cout << "/estimated_pose callback" << std::endl;
    // clear
    odom.position.x = 0.0;
    odom.position.y = 0.0;
    odom.position.z = 0.0;
    odom.orientation.x = 0.0;
    odom.orientation.y = 0.0;
    odom.orientation.z = 0.0;
    odom.orientation.w = 1.0;
    // init
    odom.position.x = estimated_pose_msg.pose.pose.position.x;
    odom.position.y = estimated_pose_msg.pose.pose.position.y;
    odom.position.z = estimated_pose_msg.pose.pose.position.z;

    init_estimated_pose = true;
}

void Odometry_Publisher::update_odometry()
{
    if (sub_linear_vel == true) {
        //std::cout << "update odometry" << std::endl;
        // calculate sampling time
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();

        // クォータニオン→オイラー角
        // https://gist.github.com/eborghi10/c3302fae021f894cda531e5d2f739ac1
        // https://myenigma.hatenablog.com/entry/20130719/1374304154#ROS%E3%81%AB%E3%81%8A%E3%81%91%E3%82%8B%E3%82%AA%E3%82%A4%E3%83%A9%E3%83%BC%E8%A7%92%E3%82%AF%E3%82%A9%E3%83%BC%E3%82%BF%E3%83%8B%E3%82%AA%E3%83%B3%E3%81%AE%E8%A8%88%E7%AE%97
        double roll, pitch, yaw;
        tf::Quaternion odom_quat(
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w
        ); 
        tf::Matrix3x3 m(odom_quat);
        m.getRPY(roll, pitch, yaw);
        
        // position X, Y, Theta
        std::cout << "odom x" << odom.position.x << std::endl;
        std::cout << "dt" << dt << std::endl;
        std::cout << "linear_x" << linear_x << std::endl;
        std::cout << "angular_z" << angular_z << std::endl;
        std::cout << "yaw" << yaw << std::endl;

        float updated_yaw;
        odom.position.x = odom.position.x + dt * linear_x * cos(yaw);
        odom.position.y = odom.position.y + dt * linear_x * sin(yaw);
        updated_yaw = yaw + angular_z * dt;
        //std::cout << odom.position.x << std::endl;

        geometry_msgs::Quaternion updated_orientation = tf::createQuaternionMsgFromYaw(updated_yaw);
        odom.orientation = updated_orientation;

        odom_pub.publish(odom);

        // publish odom_path
        geometry_msgs::PoseStamped odom_point;
        odom_point.header.frame_id = "map";
        odom_point.header.stamp = ros::Time::now();
        odom_point.pose.position.x = odom.position.x;
        odom_point.pose.position.y = odom.position.y;
        odom_point.pose.position.z = 0.0;
        odom_point.pose.orientation.w = odom.orientation.w;
        odom_point.pose.orientation.x = odom.orientation.x;
        odom_point.pose.orientation.y = odom.orientation.y;
        odom_point.pose.orientation.z = odom.orientation.z;

        odom_path.poses.push_back(odom_point);
        odom_path_pub.publish(odom_path);

        last_time = current_time;
    }

    sub_linear_vel = false;
    sub_angular_vel = false;
    init_estimated_pose = false;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "odometry_publisher");
    
    Odometry_Publisher odometry_publisher;
    ros::Rate loop_rate(50);
	while(ros::ok()){
		ros::spinOnce();
        odometry_publisher.update_odometry();
		loop_rate.sleep();
	}

    return 0;
}