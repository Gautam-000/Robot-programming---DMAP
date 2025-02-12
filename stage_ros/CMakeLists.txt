cmake_minimum_required(VERSION 2.8.3)
project(stage_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_srvs
  tf
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(stage REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${STAGE_INCLUDE_DIRS}
)

catkin_package()

add_executable(stageros src/stageros.cpp)

set(${PROJECT_NAME}_extra_libs "")
if(UNIX AND NOT APPLE)
  set(${PROJECT_NAME}_extra_libs dl)
endif()

target_link_libraries(stageros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${STAGE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(stageros ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS stageros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})