# *****************************************
add_executable(data_player data_player.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(data_player
  ${catkin_LIBRARIES}
)
# *****************************************

add_executable(odom_recorder odom_recorder.cc)
target_link_libraries(odom_recorder
  ${catkin_LIBRARIES}
)
# *****************************************
add_executable(lidar_recorder lidar_recorder.cc)

target_link_libraries(lidar_recorder
  ${catkin_LIBRARIES}
)

# *****************************************
add_executable(img_recorder img_recorder.cc)

target_link_libraries(img_recorder
  ${catkin_LIBRARIES}
)

# *****************************************
add_executable(img_player img_player.cc)

target_link_libraries(img_player
  ${catkin_LIBRARIES}
)

# *****************************************
add_executable(lidar_player lidar_player.cc)

target_link_libraries(lidar_player
  ${catkin_LIBRARIES}
)

# *****************************************
add_executable(compress_img_recorder compressed_img_recorder.cc)

target_link_libraries(compress_img_recorder
  ${catkin_LIBRARIES}
)

# *****************************************
add_executable(board_model board_model.cc)

target_link_libraries(board_model
  ${catkin_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
)
# *****************************************
add_executable(kitti_lidar_player kitti_lidar_player.cc)

target_link_libraries(kitti_lidar_player
  ${catkin_LIBRARIES}
)
# *****************************************
add_executable(kitti_bin_pcd showKittiBinPC.cc)

target_link_libraries(kitti_bin_pcd
  ${catkin_LIBRARIES}
)

# *****************************************
add_executable(oneshot_recorder oneshot_recorder.cc)

target_link_libraries(oneshot_recorder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


