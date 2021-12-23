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
#include <stdlib.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ros/ros.h"
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

string pcd_save_path =
    "/home/ramlab/Documents/publication/unifiedCali/data/simu/cam-lidar/"
    "noise-free/pcd/";
string img_save_path =
    "/home/ramlab/Documents/publication/unifiedCali/data/simu/cam-lidar/"
    "noise-free/img/";
string img_topic = "/usb_cam/image_raw";
string pcd_topic = "/velodyne_points";

/** 
 * @brief: record point cloud and images from topic message and save them to pcd and jpg files
 *  with synchronization or without synchronization.
 */

bool need_sync = false;
bool save_pcd_flag = false;
bool save_img_flag = false;

void Syncallback(const PointCloud2ConstPtr& ori_pointcloud,
                 const ImageConstPtr& ori_image) {
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  cv_bridge::CvImagePtr cv_image_ptr;
  cv::Mat cv_image;
  cv_image_ptr =
      cv_bridge::toCvCopy(ori_image, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  pcl::fromROSMsg(*ori_pointcloud, pcl_cloud);
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  //  选择jpeg
  compression_params.push_back(100);  //  在这个填入你要的图片质量
  if (save_pcd_flag) {
    save_pcd_flag = false;
    std::cout << "\033[1;32m Syn! \033[0m" << std::endl;
    std::cout << "syn pointcloud' timestamp : " << ori_pointcloud->header.stamp
              << std::endl;
    std::cout << "syn image's timestamp : " << ori_image->header.stamp
              << std::endl;
    cv::imwrite(img_save_path + "/" +
                    std::to_string(ori_image->header.stamp.toSec()) + ".jpg",
                cv_image, compression_params);
    pcl::io::savePCDFile(
        pcd_save_path + "/" +
            std::to_string(ori_pointcloud->header.stamp.toSec()) + ".pcd",
        pcl_cloud);
  }
}

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

  // ***** Load Parameters ****
  ROS_WARN("Starting synchronizer...");

  // 建立需要订阅的消息对应的订阅器
  ros::Subscriber pcd_sub;
  ros::Subscriber img_sub;
  message_filters::Subscriber<PointCloud2> PCInfo_sub;
  message_filters::Subscriber<Image> ImageInfo_sub;
  typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PCInfo_sub,
                                  ImageInfo_sub);  // queue size=10
  sync.registerCallback(boost::bind(&Syncallback, _1, _2));
  if (need_sync) {
    PCInfo_sub.subscribe(nh, pcd_topic, 1);
    ImageInfo_sub.subscribe(nh, img_topic, 1);
  } else {
    pcd_sub =
        nh.subscribe<sensor_msgs::PointCloud2>(pcd_topic, 10, Lidarcallback);
    img_sub = nh.subscribe<sensor_msgs::Image>(img_topic, 10, Imgcallback);
  }

  thread t(keyaction);
  t.detach();
  ros::spin();
  return 0;
}