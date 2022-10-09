#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

def imu_callback(imu_msg):
    ps_msg = PoseStamped()
    ps_msg.header=imu_msg.header
    ps_msg.pose.orientation=imu_msg.orientation
    posestamped_pub.publish(ps_msg)

if __name__ == "__main__":
    rospy.init_node("imu_to_pose")
    #Publisher
    posestamped_pub = rospy.Publisher("imu_pose", PoseStamped, queue_size=10)
    #Subscriber
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.spin()