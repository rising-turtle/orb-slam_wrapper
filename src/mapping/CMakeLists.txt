
project(orb_slam)

# set(LSD_SLAM_SRC "/home/davidz/work/ros_hydro_ws/src/lsd_slam/lsd_slam_core/src")
# set(LSD_SLAM_SRC "/home/etas/hxzhang/ros/indigo/src/lsd_slam_ext/lsd_slam_core/src")
set(LSD_SLAM_SRC "../../../lsd_slam_ext/lsd_slam_core/src")
message("STATUS LSD_SLAM_SRC: ${LSD_SLAM_SRC}")
include_directories(${LSD_SLAM_SRC})
include_directories(${LSD_SLAM_SRC}/../thirdparty/Sophus)

add_executable(orb_map orb_map.cpp)
target_link_libraries(orb_map lsdslam orb_wrapper ${catkin_LIBRARIES})

add_executable(kf_map recv_kf_map.cpp)
target_link_libraries(kf_map lsdslam orb_wrapper ${catkin_LIBRARIES})
