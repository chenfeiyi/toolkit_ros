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
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <thread>
#include "ros/ros.h"
#include <stdlib.h>
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

/** 
 * @brief: record two point cloud topic and one image topic without synchronization
 */

string pcd_save_path = "/home/udi/Documents/bbc/data/pcd/";
string pcd_save_path2 = "/home/udi/Documents/bbc/data/pcd2/";
string img_save_path = "/home/udi/Documents/bbc/data/img/";
string img_topic = "/usb_cam/image_raw";
string pcd_topic1 = "/ns1/velodyne_points";
string pcd_topic2 = "/ns2/velodyne_points";

bool save_pcd_flag1 = false;
bool save_pcd_flag2 = false;
bool save_img_flag = false;


void Lidarcallback(sensor_msgs::PointCloud2ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  std::string file_name;
  file_name =
      pcd_save_path + std::to_string(msg->header.stamp.toSec()) + ".pcd";
  if (save_pcd_flag1) {
    save_pcd_flag1 = false;
    pcl::io::savePCDFile(file_name, cloud_in);
    ROS_INFO("save pcd file!");
  }
}
void Lidarcallback2(sensor_msgs::PointCloud2ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);
  std::string file_name;
  file_name =
      pcd_save_path2 + std::to_string(msg->header.stamp.toSec()) + ".pcd";
  if (save_pcd_flag2) {
    save_pcd_flag2 = false;
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
    save_pcd_flag1 = true;
    save_pcd_flag2 = true;
    cin.ignore();
    cin.get();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "oneshot_record");
  ros::NodeHandle nh;

  // ***** Load Parameters ****
  ROS_WARN("Starting synchronizer...");

  // 建立需要订阅的消息对应的订阅器
  ros::Subscriber pcd_sub;
  ros::Subscriber pcd_sub2;
  ros::Subscriber img_sub;
    pcd_sub =
        nh.subscribe<sensor_msgs::PointCloud2>(pcd_topic1, 10, Lidarcallback);
    pcd_sub2 =
        nh.subscribe<sensor_msgs::PointCloud2>(pcd_topic2, 10, Lidarcallback2);
    img_sub =
        nh.subscribe<sensor_msgs::Image>(img_topic, 10, Imgcallback);

  thread t(keyaction);
  t.detach();
  ros::spin();
  return 0;
}