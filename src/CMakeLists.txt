project(orb_slam)

add_subdirectory(types)

add_library(orb_wrapper system.cpp track.cpp keyframe.cpp plane.cpp img2pc.cpp detect_plane.cpp optimizer.cpp local_map.cpp)
target_link_libraries(orb_wrapper plane cam_model orb_g2o_extend ${LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(orb-RGBD orb_rgbd.cpp)
target_link_libraries(orb-RGBD orb_wrapper ${LIBS} ${catkin_LIBRARIES})

add_executable(test_plane_f200 test_plane.cpp)
target_link_libraries(test_plane_f200 orb_wrapper ${LIBS} ${catkin_LIBRARIES})

add_subdirectory(mapping) 

add_subdirectory(test)
