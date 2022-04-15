/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-14 17:19:14
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
 * @brief: play images from files.
 * And the file name should the sequence number or the timestamp.
 */
int main(int argc, char *argv[]) {
  
  cmdline::parser a;
  a.add<std::string>("path", 'p', "data path", true);
  a.add<bool>("loop", 'l', "play data repeatly", false, false);
  a.add<float>("hz", 'z', "frequency to paly the data", false, 10);
  a.parse_check(argc, argv);
  ros::init(argc, argv, "img_player");
  ros::NodeHandle nh;

  ros::Publisher img_pub =
      nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 10);
  std::string img_file_path = a.get<std::string>("path");

  vector<string> img_file_list;
  vector<string> pcd_file_list;
  calibrator_pipeline::common::GetFilelist(img_file_path, &img_file_list,
                                           false);
  sort(img_file_list.begin(), img_file_list.end());

  ros::Rate rate(a.get<float>("hz"));
  sensor_msgs::ImagePtr img_msg;
  std::cout << "******************************" << std::endl;
  std::cout << "start to play data" << std::endl;
  int i = 0;
  while (ros::ok()) {
    std::string img_name = img_file_path + img_file_list[i];
    cv::Mat img = cv::imread(img_name);
    std::vector<std::string> img_field;
    calibrator_pipeline::common::SplitString(img_file_list[i], &img_field, '.');
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    img_msg->header.stamp =  // ros::Time::now();
        ros::Time(std::atof((img_field[0] + '.' + img_field[1]).c_str()));
    img_pub.publish(img_msg);
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
