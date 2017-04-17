project(orb_slam)

add_library(orb_g2o_extend types_plane_se3d.cpp)
target_link_libraries(orb_g2o_extend ${ORB_G2O}/lib/libg2o.so)

add_executable(test_plane3d test_plane3d.cpp)
target_link_libraries(test_plane3d orb_g2o_extend ${LIBS} ${catkin_LIBRARIES})


