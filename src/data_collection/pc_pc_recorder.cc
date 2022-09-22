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
#include "../common/cmdline.h"
#include "ros/ros.h"
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;


/** 
 * @brief: record two point cloud from ros message.
 */

string pcd_save_path1;
string pcd_save_path2;
string pcd_topic1;
string pcd_topic2;

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
  cmdline::parser a;
  a.add<std::string>("lidar_topic1", 's', "source lidar topic name", false, "/ns1/velodyne_points");
  a.add<std::string>("lidar_topic2", 't', "target lidar topic name", false, "/ns2/velodyne_points");
  a.add<std::string>("path", 'p', "save path", true);
  a.parse_check(argc, argv);
  std::string pcd_topic1 = a.get<std::string>("lidar_topic1");
  std::string pcd_topic2 = a.get<std::string>("lidar_topic2");
  std::string save_path = a.get<std::string>("path");
  pcd_save_path1=save_path+"/pcd1";
  pcd_save_path2=save_path+"/pcd2";
  DIR *dir;   
  if ((dir=opendir(save_path.c_str())) == NULL)   
  { 
    ROS_WARN("folder not exist!!");
    return 0;
  }
  if ((dir=opendir(pcd_save_path1.c_str())) == NULL)   
  { 
    std::string cmdpath ="mkdir -p "+pcd_save_path1;
    system(cmdpath.c_str());
  }
  if ((dir=opendir(pcd_save_path2.c_str())) == NULL)   
  { 
    std::string cmdpath ="mkdir -p "+pcd_save_path2;
    system(cmdpath.c_str());
  }
  // 建立需要订阅的消息对应的订阅器
  ROS_WARN("Starting recording...");
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