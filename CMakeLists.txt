cmake_minimum_required(VERSION 2.8.3)
project(toolkit_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -w -O2 ")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_player
#  CATKIN_DEPENDS cv_bridge image_transport pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ./common
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_player lib/data_player.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(data_player
  ${catkin_LIBRARIES}
)

