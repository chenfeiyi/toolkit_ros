# ********************play point cloud and images*********************
add_executable(pc_im_player_from_files pc_im_player_from_files.cc)

target_link_libraries(pc_im_player_from_files
  ${catkin_LIBRARIES}
)

# ********************play point cloud and images*********************
add_executable(pc_im_player_from_files2 pc_im_player_from_files2.cc)

target_link_libraries(pc_im_player_from_files2
  ${catkin_LIBRARIES}
)


# ********************play point cloud and images*********************
add_executable(pc_im_player_from_files3 pc_im_player_from_files3.cc)

target_link_libraries(pc_im_player_from_files3
  ${catkin_LIBRARIES}
)

# ********************play point cloud only from files*********************
add_executable(pc_player pc_player.cc)

target_link_libraries(pc_player
  ${catkin_LIBRARIES}
)

# ********************play image only from files*********************
add_executable(img_player img_player.cc)

target_link_libraries(img_player
  ${catkin_LIBRARIES}
)

# ********************play image only from files*********************
add_executable(kitti_pc_player kitti_pc_player.cc)

target_link_libraries(kitti_pc_player
  ${catkin_LIBRARIES}
)
# ********************play poses from file*********************
add_executable(pose_publisher pose_publisher.cc)

target_link_libraries(pose_publisher
  ${catkin_LIBRARIES}
)

