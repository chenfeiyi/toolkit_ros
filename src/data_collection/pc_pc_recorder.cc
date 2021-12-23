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


/** 
 * @brief: record two point cloud from ros message
 */

string pcd_save_path1 = "/home/udi/Documents/bbc/tool_ws/src/pcd1/";
string pcd_save_path2 = "/home/udi/Documents/bbc/tool_ws/src/pcd2/";
string pcd_topic1 = "/right/rslidar_points";
string pcd_topic2 = "/left/rslidar_points";

bool save_pcd_flag = false;

void Syncallback(const PointCloud2ConstPtr &ori_pointcloud1,
                 const PointCloud2ConstPtr &ori_pointcloud2) {
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud1;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud2;
  pcl::fromROSMsg(*ori_pointcloud1, pcl_cloud1);
  pcl::fromROSMsg(*ori_pointcloud2, pcl_cloud2);
  if (save_pcd_flag) {
    save_pcd_flag = false;
    std::cout << "\033[1;32m Syn! \033[0m" << std::endl;
    std::cout << "syn pointcloud' timestamp : " << ori_pointcloud1->header.stamp
              << std::endl;
    std::cout << "syn pointcloud' timestamp : " << ori_pointcloud2->header.stamp
              << std::endl;
    pcl::io::savePCDFile(
        pcd_save_path1 + "/" +
            std::to_string(ori_pointcloud1->header.stamp.toSec()) + ".pcd",
        pcl_cloud1);
    pcl::io::savePCDFile(
        pcd_save_path2 + "/" +
            std::to_string(ori_pointcloud2->header.stamp.toSec()) + ".pcd",
        pcl_cloud2);
  }
}

void keyaction() {
  while (1) {
    cout << "Press any key to save data..." << endl;
    save_pcd_flag = true;
    cin.ignore();
    cin.get();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "oneshot_record");
  ros::NodeHandle nh;

  // ***** Load Parameters ****
  ROS_WARN("Starting synchronizer...");

  // 建立需要订阅的消息对应的订阅器
  ros::Subscriber pcd_sub1;
  ros::Subscriber pcd_sub2;
  message_filters::Subscriber<PointCloud2> PCInfo_sub1(nh, pcd_topic1, 1);
  message_filters::Subscriber<PointCloud2> PCInfo_sub2(nh, pcd_topic2, 1);

  typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PCInfo_sub1,
                                  PCInfo_sub2);  // queue size=10
  sync.registerCallback(boost::bind(&Syncallback, _1, _2));

  thread t(keyaction);
  t.detach();
  ros::spin();
  return 0;
}