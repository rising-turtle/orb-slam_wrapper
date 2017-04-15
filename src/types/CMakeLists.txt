project(orb_slam)

set(ORB_G2O /home/davidz/work/3rdlibs/ORB_SLAM2-master/Thirdparty/g2o)
include_directories(${ORB_G2O} ${ORB_G2O}/g2o/types) 
link_directories(${ORB_G2O}/lib)

message("############### ORB_G2O : ${ORB_G2O}######################")

add_library(orb_g2o_extend types_plane_se3d.cpp)
target_link_libraries(orb_g2o_extend ${ORB_G2O}/lib/libg2o.so)

add_executable(test_plane3d test_plane3d.cpp)
target_link_libraries(test_plane3d orb_g2o_extend g2o ${LIBS} ${catkin_LIBRARIES})


