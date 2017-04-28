
project(orb_slam)

add_executable(show_feature test_feature.cpp)
target_link_libraries(show_feature ${LIBS} ${catkin_LIBRARIES})

add_executable(show_feature_with_mask test_feature_mask.cpp)
target_link_libraries(show_feature_with_mask ${LIBS} ${catkin_LIBRARIES})

# add_executable(orb-RGBD ros_rgbd.cc)
# target_link_libraries(orb-RGBD ${LIBS} ${catkin_LIBRARIES})



