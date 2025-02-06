add_subdirectory(rp_stuff)

add_executable(dmap_show_live_node dmap_show_live_node.cpp)
target_link_libraries(dmap_show_live_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rp_library
)

add_executable(dmap_localize_node dmap_localize_node.cpp)
target_link_libraries(dmap_localize_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rp_library
)

add_executable(dmap_tracker_node dmap_tracker_node.cpp)
target_link_libraries(dmap_tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rp_library
)
