# gnss_imu_odom_ESKF
gnss_imu_odom_ESKF

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

## STEP4 rosbag play
### Terminal1
```
cd catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch gnss_imu_odom_ESKF eskf_localization.launch
```
### Terminal 2
```
rosbag play --clock Downloads/utbm_robocar_dataset_20180719_noimage.bag
```
### Terminal3
```
source ~/catkin_ws/devel/setup.bash
rostopic echo /nmea_sentence
```
### Terminal4
```
source ~/catkin_ws/devel/setup.bash
rostopic echo /fix
```
