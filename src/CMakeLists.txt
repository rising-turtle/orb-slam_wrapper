project(orb_slam)

add_subdirectory(types)

# add_executable(orb-RGBD ros_rgbd.cc)
# target_link_libraries(orb-RGBD ${LIBS} ${catkin_LIBRARIES})

add_library(orb_wrapper system.cpp track.cpp keyframe.cpp plane.cpp img2pc.cpp detect_plane.cpp optimizer.cpp local_map.cpp)
target_link_libraries(orb_wrapper plane cam_model orb_g2o_extend ${LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(orb-RGBD orb_rgbd.cpp)
target_link_libraries(orb-RGBD orb_wrapper ${LIBS} ${catkin_LIBRARIES})

add_executable(test_plane_f200 test_plane.cpp)
target_link_libraries(test_plane_f200 orb_wrapper ${LIBS} ${catkin_LIBRARIES})

add_executable(show_feature test_feature.cpp)
target_link_libraries(show_feature ${LIBS} ${catkin_LIBRARIES})

add_subdirectory(mapping) 


