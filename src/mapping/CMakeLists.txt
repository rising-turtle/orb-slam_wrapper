
project(orb_slam)

set(LSD_SLAM_SRC "/home/davidz/work/ros_hydro_ws/src/lsd_slam/lsd_slam_core/src")
message("STATUS LSD_SLAM_SRC: ${LSD_SLAM_SRC}")
include_directories(${LSD_SLAM_SRC})
include_directories(${LSD_SLAM_SRC}/../thirdparty/Sophus)

add_executable(orb_map orb_map.cpp)
target_link_libraries(orb_map lsdslam orb_wrapper ${catkin_LIBRARIES})
