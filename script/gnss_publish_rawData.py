#! /usr/bin/env python3
import pandas as pd

# import for ros function
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix

class GNSS_Publish_Rawdata():
    def __init__(self):
        rospy.init_node("gnss_publish_rawdata", anonymous=True)

        # get pose data from csv file
        self.csv_data = pd.read_csv("~/catkin_ws/src/gnss_imu_odom_ESKF/gnss_path/gnss_rawData1.csv")
        #self.csv_data = pd.read_csv("~/catkin_ws/src/gnss_imu_odom_ESKF/gnss_path/gnss_rawData2.csv")
        self.pose_list = self.get_poses_from_csvdata()
        
        # creat path data
        """
        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        self.path.poses = self.pose_list
        """

        # initialize publisher
        self.path_pub = rospy.Publisher("/path", Path, queue_size = 50)
        self.path_num_pub = rospy.Publisher("/path_num", Int32, queue_size = 10)
        self.gnss_pub = rospy.Publisher("/fix", NavSatFix, queue_size = 50)

    def get_poses_from_csvdata(self):
        poses_list = []
        for index in range(len(self.csv_data)):
            temp_pose = NavSatFix()
            temp_pose.header.seq = index
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.header.frame_id = "navsat"
            temp_pose.latitude = self.csv_data["latitude"][index]
            temp_pose.longitude = self.csv_data["longitude"][index]
            temp_pose.altitude = self.csv_data["altitude"][index]

            poses_list.append(temp_pose)

        return poses_list

    def publish_path(self, index):
        #self.path_pub.publish(self.path)
        self.path_num_pub.publish(len(self.csv_data))
        self.gnss_pub.publish(self.pose_list[index])

if __name__ == '__main__':
    print('Path Publisher is Started...')
    path_publisher = GNSS_Publish_Rawdata()
    
    index = 0
    r = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        if index < len(path_publisher.csv_data):
            path_publisher.publish_path(index)
            index = index + 1
        r.sleep()
    print("Path Publisher finished!")