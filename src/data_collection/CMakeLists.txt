# ******************record two image at the same time***********************
add_executable(im_im_recorder im_im_recorder.cc)

target_link_libraries(im_im_recorder
  ${catkin_LIBRARIES}
)

# *******************record compressed image**********************
add_executable(compressed_im_recorder compressed_im_recorder.cc)

target_link_libraries(compressed_im_recorder
  ${catkin_LIBRARIES}
)

# *******************record image**********************
add_executable(img_recorder img_recorder.cc)

target_link_libraries(img_recorder
  ${catkin_LIBRARIES}
)

# *******************record pose message into txt file**********************
add_executable(odom_recorder odom_recorder.cc)

target_link_libraries(odom_recorder
  ${catkin_LIBRARIES}
)

# *******************record point cloud and image from ros message**********************
add_executable(pc_im_recorder pc_im_recorder.cc)

target_link_libraries(pc_im_recorder
  ${catkin_LIBRARIES}
)
# *******************record point cloud and image from ros message**********************
add_executable(pc_im_synchronizer pc_im_synchronizer.cc)

target_link_libraries(pc_im_synchronizer
  ${catkin_LIBRARIES}
)

# *******************record customized point cloud **********************
add_executable(pc_record_custom pc_record_custom.cc)

target_link_libraries(pc_record_custom
  ${catkin_LIBRARIES}
)

# *******************record point cloud **********************
add_executable(pc_recorder pc_recorder.cc)

target_link_libraries(pc_recorder
  ${catkin_LIBRARIES}
)


# *******************record two point cloud **********************
add_executable(pc_pc_recorder pc_pc_recorder.cc)

target_link_libraries(pc_pc_recorder
  ${catkin_LIBRARIES}
)

# *******************record two point cloud and one image**********************
add_executable(pc_pc_im_recorder pc_pc_im_recorder.cc)

target_link_libraries(pc_pc_im_recorder
  ${catkin_LIBRARIES}
)

# *******************record two point cloud and one image**********************
add_executable(livox_map_recorder livox_map_recorder.cc)

target_link_libraries(livox_map_recorder
  ${catkin_LIBRARIES}
)
add_executable(livox_map_cam_recorder liv_map_cam_recorder.cc)

target_link_libraries(livox_map_cam_recorder
  ${catkin_LIBRARIES}
)


