/**
 * @file:
 * @brief:
 * @details:
 * @author: CHEN Feiyi
 * @email: chenfeiyi@unity-drive.com
 * @version: 1.0.0
 * @licence: Copyright 2021 Unity-Drive Inc. All rights reserved
 * @date:
 */
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <thread>
#include <vector>
#include <dirent.h>

#include "../common/cmdline.h"
#include "ros/ros.h"
using namespace sensor_msgs;
using namespace std;

string pcd_save_path ="";
string img_save_path ="";

/** 
 * @brief: record point cloud and images from topic message and save them to pcd and jpg files
 *  with synchronization or without synchronization.
 */

bool need_sync = false;
bool save_pcd_flag = false;
bool save_img_flag = false;
void Lidarcallback(sensor_msgs::PointCloud2ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  std::string file_name;
  file_name =
      pcd_save_path + std::to_string(msg->header.stamp.toSec()) + ".pcd";
  if (save_pcd_flag) {
    save_pcd_flag = false;
    pcl::io::savePCDFile(file_name, cloud_in);
    ROS_INFO("save pcd file!");
  }
}

void Imgcallback(sensor_msgs::ImageConstPtr msg) {
  cv_bridge::CvImagePtr cv_image_ptr;
  cv::Mat cv_image;
  cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  std::string file_name;
  file_name =
      img_save_path + std::to_string(msg->header.stamp.toSec()) + ".jpg";
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  if (save_img_flag) {
    save_img_flag = false;
    cv::imwrite(file_name, cv_image, compression_params);
    ROS_INFO("save img file!");
  }
}

void keyaction() {
  while (1) {
    cout << "Press any key to save data..." << endl;
    save_img_flag = true;
    save_pcd_flag = true;
    cin.ignore();
    cin.get();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "oneshot_record");
  ros::NodeHandle nh;
  cmdline::parser a;
  a.add<std::string>("cam_topic", 'c', "camera topic name", false, "/usb_cam/image_raw");
  a.add<std::string>("lidar_topic", 'l', "lidar topic name", false, "/velodyne_points");
  a.add<std::string>("path", 'p', "save path", true);
  a.parse_check(argc, argv);
  std::string pcd_topic = a.get<std::string>("lidar_topic");
  std::string img_topic = a.get<std::string>("cam_topic");
  std::string save_path = a.get<std::string>("path");
  img_save_path=save_path+"/img/";
  pcd_save_path=save_path+"/pcd/";
  DIR *dir;   
  if ((dir=opendir(save_path.c_str())) == NULL)   
  { 
    ROS_WARN("folder not exist!!");
    return 0;
  }
  if ((dir=opendir(img_save_path.c_str())) == NULL)   
  { 
    std::string cmdpath ="mkdir -p "+img_save_path;
    system(cmdpath.c_str());
  }
  if ((dir=opendir(pcd_save_path.c_str())) == NULL)   
  { 
    std::string cmdpath ="mkdir -p "+pcd_save_path;
    system(cmdpath.c_str());
  }
  // 建立需要订阅的消息对应的订阅器
  ros::Subscriber pcd_sub;
  ros::Subscriber img_sub;
  
  pcd_sub =
      nh.subscribe<sensor_msgs::PointCloud2>(pcd_topic, 10, Lidarcallback);
  img_sub = nh.subscribe<sensor_msgs::Image>(img_topic, 10, Imgcallback);
  thread t(keyaction);
  t.detach();
  ros::spin();
  return 0;
}