#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

#include <boost/date_time.hpp>

class Odom_Publisher
{
    private:
        ros::NodeHandle nh;
        ros::Publisher odom_pub;
        ros::Subscriber cmd_vel_sub;
        ros::Subscriber estimated_sub;
        // timet set
        ros::Time current_time = ros::Time::now();
        ros::Time last_time = ros::Time::now();
        float dt = (current_time - last_time).toSec();
        // odometry
        nav_msgs::Odometry odom;
    public:
        Odom_Publisher();
        ~Odom_Publisher();
        // cmd_vel callback
        void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel_msg);
        float linear_x  = 0.0;  //[m/s]
        float linear_y  = 0.0;  //[m/s]
        float angular_z = 0.0;  //[rad/s]
        // estimated_pose callback
        void estimated_pose_callback(const geometry_msgs::Pose $estimated_pose_msg);
        // odometry publish
        void update_odometry();
};

Odom_Publisher::Odom_Publisher()
{
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    cmd_vel_sub = nh.subscribe("/vel", 10, &Odom_Publisher::cmd_vel_callback, this);
    estimated_sub = nh.subscribe("/estimated_pose", 10, &Odom_Publisher::estimated_pose_callback, this);

    // odometry
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
}

Odom_Publisher::~Odom_Publisher()
{
    std::cout << "Finish Odom Publisher!" << std::endl;
}

void Odom_Publisher::cmd_vel_callback(const geometry_msgs::Twist &cmd_vel_msg)
{
    linear_x  = cmd_vel_msg.linear.x;  //[m/s]
    linear_y  = cmd_vel_msg.linear.y;  //[m/s]
    angular_z = cmd_vel_msg.angular.z; //[rad/s]
}

void Odom_Publisher::update_odometry()
{
    // calculate sampling time
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    //std::cout<< dt << std::endl;

    // クォータニオン→オイラー角
    // https://gist.github.com/eborghi10/c3302fae021f894cda531e5d2f739ac1
    // https://myenigma.hatenablog.com/entry/20130719/1374304154#ROS%E3%81%AB%E3%81%8A%E3%81%91%E3%82%8B%E3%82%AA%E3%82%A4%E3%83%A9%E3%83%BC%E8%A7%92%E3%82%AF%E3%82%A9%E3%83%BC%E3%82%BF%E3%83%8B%E3%82%AA%E3%83%B3%E3%81%AE%E8%A8%88%E7%AE%97
    double roll, pitch, yaw;
    tf::Quaternion odom_quat(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    ); 
    tf::Matrix3x3 m(odom_quat);
    m.getRPY(roll, pitch, yaw);
    //std::cout<< yaw << std::endl;
    
    // position X, Y, Theta
    float updated_yaw;
    odom.pose.pose.position.x = odom.pose.pose.position.x + dt * linear_x * cos(yaw);
    odom.pose.pose.position.y = odom.pose.pose.position.y + dt * linear_x * sin(yaw);
    updated_yaw = yaw + angular_z * dt;

    geometry_msgs::Quaternion updated_orientation = tf::createQuaternionMsgFromYaw(updated_yaw);
    odom.pose.pose.orientation = updated_orientation;
   
    // update odometry
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom_pub.publish(odom);

    /*************************************/
    /* map to base_link for mobile_robot */
    /*************************************/
    // update /map to /odom transform
    geometry_msgs::Quaternion map_to_odom_quat = tf::createQuaternionMsgFromYaw(0.0);
    map_to_odom_trans.header.stamp = current_time;
    map_to_odom_trans.header.frame_id = "map";
    map_to_odom_trans.child_frame_id = "odom";
    map_to_odom_trans.transform.translation.x = 0.0;
    map_to_odom_trans.transform.translation.y = 0.0;
    map_to_odom_trans.transform.translation.z = 0.0;
    map_to_odom_trans.transform.rotation = map_to_odom_quat;
    map_to_odom_broadcaster.sendTransform(map_to_odom_trans);

    // update /odom to /base_link transform
    odom_to_baselink_trans.header.stamp = current_time;
    odom_to_baselink_trans.header.frame_id = "odom";
    odom_to_baselink_trans.child_frame_id = "base_link";
    odom_to_baselink_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_to_baselink_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_to_baselink_trans.transform.translation.z = odom.pose.pose.position.z;
    odom_to_baselink_trans.transform.rotation = updated_orientation;
    odom_to_baselink_broadcaster.sendTransform(odom_to_baselink_trans);
    
    last_time = current_time;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "odom_publisher");
    
    Odom_Publisher odom_publisher;
    ros::Rate loop_rate(50);
	while(ros::ok()){
		ros::spinOnce();
        odom_publisher.update_odometry();
		loop_rate.sleep();
	}

    return 0;
}