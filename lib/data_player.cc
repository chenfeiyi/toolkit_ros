/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-11 21:48:40
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

#include "file_operation.hpp"
#include "cmdline.h"
using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "data_player");
  ros::NodeHandle nh;

  cmdline::parser a;
  a.add<bool>("loop", 'l', "play data repeatly", false, false);
  a.parse_check(argc, argv);

  ros::Publisher img_pub =
      nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 10);
  ros::Publisher pcd_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
  std::string img_file_path =
      "/home/ramlab/Documents/CornerSeg/dataset/targetless_calibration/img2/";
  std::string pcd_file_path =
      "/home/ramlab/Documents/CornerSeg/dataset/targetless_calibration/pcd2/";

  vector<string> img_file_list;
  vector<string> pcd_file_list;
  calibrator_pipeline::common::GetFilelist(img_file_path, &img_file_list,
                                           false);
  calibrator_pipeline::common::GetFilelist(pcd_file_path, &pcd_file_list,
                                           false);

  sort(img_file_list.begin(), img_file_list.end());
  sort(pcd_file_list.begin(), pcd_file_list.end());

  assert(img_file_list.size() == pcd_file_list.size());
  ros::Rate rate(10);
  sensor_msgs::ImagePtr img_msg;
  sensor_msgs::PointCloud2 pcd_msg;
  std::cout << "******************************" << std::endl;
  std::cout << "start to play data" << std::endl;
  int i = 0;
  while (ros::ok()) {
      std::string img_name = img_file_path + img_file_list[i];
      std::string pcd_name = pcd_file_path + pcd_file_list[i];
      cv::Mat img = cv::imread(img_name);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::io::loadPCDFile(pcd_name, *cloud_raw);

      pcl::toROSMsg(*cloud_raw, pcd_msg);
      pcd_msg.header.frame_id = "velodyne";
      img_msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

      img_pub.publish(img_msg);
      pcd_pub.publish(pcd_msg);
      rate.sleep();
      i++;
      i = i % img_file_list.size();
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
