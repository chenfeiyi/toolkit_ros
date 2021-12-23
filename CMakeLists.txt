cmake_minimum_required(VERSION 2.8.3)
project(toolkit_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -w -O2 ")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  camera_info_manager
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
find_package( Eigen3 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( apriltag REQUIRED )
set( GLOBAL_INCLUDE_DISR
  include
  ./common
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${GLOBAL_INCLUDE_DISR}

)
add_subdirectory(src)
add_subdirectory(common)


