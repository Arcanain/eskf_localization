cmake_minimum_required(VERSION 3.0.2)
project(gnss_imu_odom_ESKF)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gnss_imu_odom_ESKF
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../../devel/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(eskf_localization src/eskf_localization.cpp)
target_link_libraries(eskf_localization ${catkin_LIBRARIES})

add_executable(gps_trajectory_plotter src/gps_trajectory_plotter.cpp)
target_link_libraries(gps_trajectory_plotter ${catkin_LIBRARIES})

add_executable(odometry_publisher src/odometry_publisher.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})