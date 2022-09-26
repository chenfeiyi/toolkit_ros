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
bool flag_livox = false;
double start_time;
std::stringstream pcd_stamp;
/** 
 * @brief: record point cloud from topic message and save it to pcd files.
 * accumulate multiple frames of data to a single pcd file.  
 */
pcl::PointCloud<pcl::PointXYZI> pc_map;
float elapsed_seconds=0;
void Lidarcallback(sensor_msgs::PointCloud2ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  pc_map+=cloud_in;
  if (!flag_livox) {
    flag_livox = true;
    pcd_stamp << std::fixed << std::setprecision(6)
              << msg->header.stamp.toSec();
    start_time = msg->header.stamp.toSec();
  }
  elapsed_seconds = msg->header.stamp.toSec() - start_time;
}
int main(int argc, char** argv) {
  pcd_save_path = "";
  cmdline::parser a;
  a.add<std::string>("topic", 't', "topic name", false, "/livox/lidar");
  a.add<std::string>("path", 's', "save path", true);
  a.add<int>("period", 'p', "period", false, 6);
  a.parse_check(argc, argv);
  pc_map.clear();
  pcd_save_path = a.get<std::string>("path");
  float period = a.get<float>("period");
  ros::init(argc, argv, "data_recorder");
  ros::NodeHandle nh;
  std::string topic_name = a.get<std::string>("topic");
  ros::Subscriber pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      topic_name, 10, Lidarcallback);
  while (ros::ok()) {
    if (elapsed_seconds > period) {
      break;
    }
    ros::spinOnce();
  }

  std::string file_name;
  file_name = pcd_save_path + "/" + pcd_stamp.str() + ".pcd";
  pcl::io::savePCDFile(file_name, pc_map);
  ROS_INFO("save pcd file!");
  return 0;
}