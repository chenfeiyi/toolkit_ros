/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-14 17:24:13
 * @Description: content
 */
#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <string>

#include "cmdline.h"
#include "file_operation.hpp"
using namespace std;
/** 
 * @brief: play point cloud from files.
 * And the file name should the sequence number or the timestamp.
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lidar_player");
  ros::NodeHandle nh;

  cmdline::parser a;
  a.add<bool>("loop", 'l', "play data repeatly", false, false);
  a.add<float>("hz", 'z', "frequency to paly the data", false, 10);
  a.parse_check(argc, argv);

  ros::Publisher pcd_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
  std::string pcd_file_path =
      "/media/ramlab/hdd1/bags/targetless/targetless2/pcd/";

  vector<string> pcd_file_list;
  calibrator_pipeline::common::GetFilelist(pcd_file_path, &pcd_file_list,
                                           false);

  sort(pcd_file_list.begin(), pcd_file_list.end());

  ros::Rate rate(a.get<float>("hz"));
  sensor_msgs::PointCloud2 pcd_msg;
  std::cout << "******************************" << std::endl;
  std::cout << "start to play data" << std::endl;
  int i = 0;
  while (ros::ok()) {
    std::string pcd_name = pcd_file_path + pcd_file_list[i];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcd_name, *cloud_raw);

    pcl::toROSMsg(*cloud_raw, pcd_msg);

    std::vector<std::string> pcd_field;
    calibrator_pipeline::common::SplitString(pcd_file_list[i], &pcd_field, '.');
    pcd_msg.header.frame_id = "velodyne";
    pcd_msg.header.stamp =
        ros::Time(std::atof((pcd_field[0] + '.' + pcd_field[1]).c_str()));
    pcd_pub.publish(pcd_msg);
    rate.sleep();
    i++;
    i = i % pcd_file_list.size();
    if (i == 0) std::cout << "Done! All data have been played!" << std::endl;
    if (i == 0 && !a.get<bool>("loop")) {
      break;

    } else if (i == 0) {
      std::cout << "******************************" << std::endl;
      std::cout << "start to replay data" << std::endl;
    }
  }
  return 0;
}
