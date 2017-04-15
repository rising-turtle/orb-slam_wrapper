cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_filters
  tf
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sr_slam
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf  
  DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -fPIC -fpermissive -std=c++0x -w")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


find_package(OpenCV 2.4.3 REQUIRED)
# find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)


################## Eigen ##################
include_directories(/usr/include/eigen3)

# PCL #######################
 set(PCL_DIR "/home/davidz/work/3rdlibs/pcl_1.7.1_src/pcl-pcl-1.7.1/build")
 find_package(PCL 1.7 REQUIRED COMPONENTS common io sample_consensus segmentation visualization kdtree)
 include_directories(${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 # add_definitions(${PCL_DEFINITIONS})

###################  ORB-SLAM   ######################
set(ORB_SLAM_DIR "/home/davidz/work/3rdlibs/ORB_SLAM2-master")

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_SLAM_DIR}/Thirdparty/g2o/lib/libg2o.so
${ORB_SLAM_DIR}/lib/libORB_SLAM2.so
)

include_directories(
  ${ORB_SLAM_DIR}
  ${ORB_SLAM_DIR}/include
  ${Pangolin_INCLUDE_DIRS}
)

add_subdirectory(src)

