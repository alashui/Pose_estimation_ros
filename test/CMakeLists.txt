add_executable( pose_txt_generate pose_txt_generate.cpp )
target_link_libraries( pose_txt_generate localization ${DBoW3_LIBS} )

add_executable( image_retrieve_test image_retrieve_test.cpp )
target_link_libraries( image_retrieve_test localization ${OpenCV_LIBS} ${DBoW3_LIBS} )

add_executable( map_generate_test map_generate_test.cpp )
target_link_libraries( map_generate_test localization  ${DBoW3_LIBS})

add_executable( localization_test localization_test.cpp )
target_link_libraries( localization_test localization  ${DBoW3_LIBS} ${catkin_LIBRARIES})






find_package(PCL REQUIRED)

## Add include directories
include_directories(include
	  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_executable( obstacle_detector obstacle_detector.cpp )
target_link_libraries( obstacle_detector ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable( motion_controller_test motion_controller_test.cpp )
target_link_libraries( motion_controller_test ${PCL_LIBRARIES} ${catkin_LIBRARIES})

