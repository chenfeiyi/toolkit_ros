/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-17 15:37:29
 * @Description: content
 */
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "../common/cmdline.h"
#include "ros/ros.h"
std::string pcd_save_path;
/** 
 * @brief: record point cloud from topic message and save it to pcd files
 */
void Lidarcallback(sensor_msgs::PointCloud2ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  std::string file_name;
  file_name = pcd_save_path +
              std::to_string(msg->header.stamp.toSec()) + ".pcd";
  pcl::io::savePCDFile(file_name, cloud_in);
  ROS_INFO("save pcd file!");
}
int main(int argc, char** argv) {
  pcd_save_path = "";
  cmdline::parser a;
  a.add<std::string>("topic", 't', "topic name", false, "/velodyne_points");
  a.add<std::string>(
      "path", 'p', "save path", true);
  a.parse_check(argc, argv);
  pcd_save_path = a.get<std::string>("path");
  ros::init(argc, argv, "data_recorder");
  ros::NodeHandle nh;
  std::string topic_name = a.get<std::string>("topic");
  ros::Subscriber pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      topic_name, 10, Lidarcallback);
  ros::spin();
}