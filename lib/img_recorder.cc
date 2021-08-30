/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-14 17:19:07
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

#include "ros/ros.h"
std::string img_save_path;
void Imgcallback(sensor_msgs::ImageConstPtr msg) {
  cv_bridge::CvImagePtr cv_image_ptr;
  cv::Mat cv_image;
  cv_image_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  std::string file_name;
  file_name = img_save_path + std::to_string(msg->header.stamp.toSec())+".jpg";
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  cv::imwrite(file_name, cv_image, compression_params);
  ROS_INFO("save img file!");
}
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "img_recorder");
  ros::NodeHandle nh;
  img_save_path = "/media/ramlab/hdd1/bags/targetless/targetless1/img/";
  ros::Subscriber img_sub =
        nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 10, Imgcallback);
  ros::spin();
  return 0;
}
