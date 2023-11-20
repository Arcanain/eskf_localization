# gnss_imu_odom_ESKF
gnss imu odometry sensor fusion localization by ESKF(output NED pose)  

- [x] gnss imu sensor fusion localization by ESKF  
- [ ] gnss imu odometry sensor fusion localization by ESKF  

# Environment
OS : Ubuntu MATE with Raspberry pi4(8GB)  
ROS : noetic

# Input Output(IO)
## Input
- nmea messages /nmea_sentence(nmea_msgs/Sentence)
- gps messages /fix(sensor_msgs/NavSatFix)
- imu messages /imu/data(sensor_msgs/Imu)
- odom messages /odom(geometry_msgs/Pose)

## Output
- estimatid_pose messages /estimatid_pose(geometry_msgs/Pose)
- estimatid_path messages /estimatid_path(nav_msgs/Path)

# Software architecture
- gps trajectory plotter
![gps_plotter](https://user-images.githubusercontent.com/52307432/188691411-7d640b93-8ab3-4fd3-abe1-8a0d8c39ab70.jpg)

- imu gnss sensor fusion
![eskf_localization3](https://user-images.githubusercontent.com/52307432/188689530-40379483-06ed-4476-a668-14b6903ddda0.jpg)

- odom imu gnss sensor fusion
![eskf_localization5](https://user-images.githubusercontent.com/52307432/188694828-6f0ffb00-2a26-43f3-9a96-5f02706c8afe.jpg)

# Class Diagram

```mermaid
classDiagram
    class IMU_Data {
        double timestamp
        Eigen::Vector3d acc
        Eigen::Vector3d gyro
    }
    class GPS_Data {
        double timestamp
        Eigen::Vector3d lla
        Eigen::Vector3d ned
    }
    class State {
        double timestamp
        Eigen::Vector3d position
        Eigen::Vector3d velocity
        Eigen::Quaterniond quaternion
        Eigen::Vector3d acc_bias
        Eigen::Vector3d gyro_bias
        Eigen::Vector3d gravity
        Eigen::Matrix<double, 18, 18> PEst
        Eigen::Matrix<double, 18, 1> error
    }
    class map_projection_reference {
        uint64_t timestamp
        double lat_rad
        double lon_rad
        double sin_lat
        double cos_lat
        bool init_done
    }
    class GEOGRAPHY {
        -map_projection_reference* ref
        +GEOGRAPHY()
        +~GEOGRAPHY()
        +int map_projection_init(map_projection_reference*, double, double)
        +int map_projection_project(map_projection_reference*, double, double, float*, float*)
        +double constrain(double, double, double)
        +bool lla2enu(double*, const double*, const double*)
        +bool lla2xyz(double*, const double*)
        +bool xyz2enu(double*, const double*, const double*)
        +void rot3d(double R[3][3], const double, const double)
        +void matrixMultiply(double C[3][3], const double A[3][3], const double B[3][3])
        +void matrixMultiply(double c[3], const double A[3][3], const double b[3])
        +void rot(double R[3][3], const double, const int)
    }
    class ESKF {
        -State x
        +ESKF()
        +~ESKF()
        +void Init(const GPS_Data&, State&)
        +void Predict(const IMU_Data&, State&)
        +void Correct(const GPS_Data&, State&)
        +void State_update(State&)
        +void Error_State_Reset(State&)
        +Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond&, const Eigen::Quaterniond&)
        +Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d&)
        +Eigen::Quaterniond euler_to_quatertion(Eigen::Vector3d)
        +Eigen::Matrix<double, 18, 18> calcurate_Jacobian_Fx(Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, const double)
        +Eigen::Matrix<double, 18, 12> calcurate_Jacobian_Fi()
        +Eigen::Matrix<double, 12, 12> calcurate_Jacobian_Qi(const double)
        +Eigen::Matrix<double, 3, 18> calcurate_Jacobian_H(State&)
        +Eigen::Quaterniond getQuaFromAA(Eigen::Vector3d)
        +Eigen::Matrix<double, 3, 3> getRotFromAA(Eigen::Vector3d)
    }
    class ROS_Interface {
        -ros::NodeHandle nh
        -bool init
        -ros::Publisher gps_path_pub
        -ros::Publisher estimated_path_pub
        -ros::Publisher estimated_pose_pub
        -ros::Subscriber gps_sub
        -ros::Subscriber imu_sub
        -tf::TransformBroadcaster odom_to_baselink_broadcaster
        -geometry_msgs::TransformStamped odom_to_baselink_trans
        -nav_msgs::Path gps_path
        -nav_msgs::Path estimated_path
        -nav_msgs::Odometry estimated_pose
        -State x
        -IMU_Data imu_data
        -GPS_Data gps_data
        -map_projection_reference map_ref
        -double lat0
        -double lon0
        -double alt0
        -ESKF eskf
        -GEOGRAPHY geography
        +ROS_Interface(ros::NodeHandle&, double, double)
        +~ROS_Interface()
        +void gps_callback(const sensor_msgs::NavSatFixConstPtr&)
        +void imu_callback(const sensor_msgs::ImuConstPtr&)
        +void data_conversion_imu(const sensor_msgs::ImuConstPtr&, IMU_Data&)
        +void data_conversion_gps(const sensor_msgs::NavSatFixConstPtr&, GPS_Data&)
    }
    ROS_Interface --|> ESKF: Uses
    ROS_Interface --|> GEOGRAPHY: Uses
    ROS_Interface --|> State: Uses
    ROS_Interface --|> IMU_Data: Uses
    ROS_Interface --|> GPS_Data: Uses
    ROS_Interface --|> map_projection_reference: Uses
    ESKF --|> State: Uses
    ESKF --|> IMU_Data: Uses
    ESKF --|> GPS_Data: Uses
    GEOGRAPHY --|> map_projection_reference: Uses
```

# nmea_navsat_driver
## STEP1  Install nmea_msgs

```
cd catkin_ws/
cd src/
git clone https://github.com/ros-drivers/nmea_msgs
catkin_make
source ~/catkin_ws/devel/setup.bash
```
## STEP2 Install nmea_navsat_driver

```
cd catkin_ws/
cd src/
git clone https://github.com/ros-drivers/nmea_navsat_driver
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## STEP3 Install rosbag file

https://epan-utbm.github.io/utbm_robocar_dataset/

utbm_robocar_dataset_20190131_noimage.bag (1.5 GB) 

2019-01-31 (Fri, snow) 	08:54-09:10 (15'59") 	1 × Velodyne / ibeo / SICK / IMU / GPS / Bumblebee XB3 / fisheye

## STEP4 rosbag play
### Terminal1
```
cd catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch gnss_imu_odom_ESKF eskf_localization.launch
```
### Terminal 2
```
rosbag play --clock Downloads/utbm_robocar_dataset_20190131_noimage.bag
```
### Terminal3
```
cd catkin_ws/
source ~/catkin_ws/devel/setup.bash
rostopic echo /nmea_sentence
```
### Terminal4
```
cd catkin_ws/
source ~/catkin_ws/devel/setup.bash
rostopic echo /fix
```

# gps_trajectory_plotter
```
cd catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch gnss_imu_odom_ESKF gps_trajectory_plotter.launch 
```
### result in rviz
![Screenshot at 2022-09-04 19-30-24](https://user-images.githubusercontent.com/52307432/188315228-ccd0601e-685d-416e-8459-928097209381.png)

# eskf_localization
## gnss imu sensor fusion
- [x] gnss imu sensor fusion localization by ESKF  
```
cd catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch gnss_imu_odom_ESKF imu_gnss_eskf_localization.launch
```
### result in rviz
- the green path is made by raw GPS
- the blue path is made by ESKF

![Screenshot at 2022-09-04 23-22-30](https://user-images.githubusercontent.com/52307432/188319301-f849a459-7d64-40b3-9e6a-396494ae1cc7.png)

## gnss imu odom sensor fusion 
- [ ] gnss imu odom sensor fusion localization by ESKF  
```
cd catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch gnss_imu_odom_ESKF odom_imu_gnss_eskf_localization.launch
```
### result in rviz
- the green path is made by raw GPS
- the red path is made by raw Odometry
- the blue path is made by ESKF

![Screenshot at 2022-09-06 23-43-02](https://user-images.githubusercontent.com/52307432/188684954-7c1e0a77-91da-4f79-bac3-b0862a02e7fe.png)

### problem section with outlier
Todo outlier fix
![Screenshot at 2022-09-06 23-41-53](https://user-images.githubusercontent.com/52307432/188685227-1558ae77-4b2f-4e2b-bec4-9ad1e7a14785.png)

# Tsukuba Challenge Environment
## STEP1
```
roslaunch gnss_imu_odom_ESKF gnss_convert_path.launch 
```

## STEP2 
```
roslaunch gnss_imu_odom_ESKF gnss_path_publish.launch
```

![Screenshot at 2022-10-11 22-11-29](https://user-images.githubusercontent.com/52307432/195100309-4a39ade5-aff9-4eea-b57d-529b6c067527.png)

