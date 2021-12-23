# ******************extract data from bag file***********************
add_executable(bag_extractor bag_extractor.cc)

target_link_libraries(bag_extractor
  lib_bagreader
  ${catkin_LIBRARIES}
)

 # ********PnP solver to calibrate extrinsic parameters given 2D-3D point*******
add_executable(pnp_solver pnp_solver.cc)

target_link_libraries(pnp_solver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# ******************convert kitti point cloud from bin file to pcd file***********************
add_executable(cov_kitti_bin2pc cov_kitti_bin2pc.cc)

target_link_libraries(cov_kitti_bin2pc
  ${catkin_LIBRARIES}
)
