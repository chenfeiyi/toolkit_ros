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
 * @brief: record two image topic message and save them to jpg files
 *  with synchronization or without synchronization.
 */


string img_save_path;
string img_save_path2;
string img_topic;
string img_topic2;

bool need_sync = true;
bool save_img_flag = false;
bool save_img_flag2 = false;

void Syncallback(const ImageConstPtr& ori_image,
                 const ImageConstPtr& ori_image2) {
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  cv_bridge::CvImagePtr cv_image_ptr;
  cv_bridge::CvImagePtr cv_image_ptr2;
  cv::Mat cv_image;
  cv::Mat cv_image2;
  cv_image_ptr =
      cv_bridge::toCvCopy(ori_image, sensor_msgs::image_encodings::BGR8);
  cv_image_ptr2 =
      cv_bridge::toCvCopy(ori_image2, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  cv_image2 = cv_image_ptr2->image;
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  //  选择jpeg
  compression_params.push_back(100);  //  在这个填入你要的图片质量
  if (save_img_flag) {
    save_img_flag = false;
    std::cout << "\033[1;32m Syn! \033[0m" << std::endl;
    std::cout << "syn image's timestamp : " << ori_image->header.stamp
              << std::endl;
    std::cout << "syn image's timestamp : " << ori_image2->header.stamp
              << std::endl;
    cv::imwrite(img_save_path + "/" +
                    std::to_string(ori_image->header.stamp.toSec()) + ".jpg",
                cv_image, compression_params);
    cv::imwrite(img_save_path2 + "/" +
                    std::to_string(ori_image2->header.stamp.toSec()) + ".jpg",
                cv_image2, compression_params);
  }
}

void Imgcallback(sensor_msgs::ImageConstPtr msg) {
  cv_bridge::CvImagePtr cv_image_ptr;
  cv::Mat cv_image;
  cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  std::string file_name;
  file_name =
      img_save_path +"/"+ std::to_string(msg->header.stamp.toSec()) + ".jpg";
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  if (save_img_flag) {
    save_img_flag = false;
    cv::imwrite(file_name, cv_image, compression_params);
    std::string info = "save " + std::to_string(msg->header.stamp.toSec()) + ".jpg";
    ROS_INFO(info.c_str());
  }
}

void Imgcallback2(sensor_msgs::ImageConstPtr msg) {
  cv_bridge::CvImagePtr cv_image_ptr;
  cv::Mat cv_image;
  cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv_image = cv_image_ptr->image;
  std::string file_name;
  file_name =
      img_save_path2 + "/"+std::to_string(msg->header.stamp.toSec()) + ".jpg";
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  if (save_img_flag2) {
    save_img_flag2 = false;
    cv::imwrite(file_name, cv_image, compression_params);
    std::string info = "save " + std::to_string(msg->header.stamp.toSec()) + ".jpg";
    ROS_INFO(info.c_str());
  }
}

void keyaction() {
  while (1) {
    cout << "Press any key to save data..." << endl;
    save_img_flag = true;
    save_img_flag2 = true;
    cin.ignore();
    cin.get();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "oneshot_record");
  ros::NodeHandle nh;

  // ***** Load Parameters ****
  cmdline::parser a;
  a.add<std::string>("camera_topic1", 's', "source camera topic name", false, "/ns1/usb_cam/image_raw");
  a.add<std::string>("camera_topic2", 't', "target camera topic name", false, "/ns2/usb_cam/image_raw");
  a.add<std::string>("path", 'p', "save path", true);
  a.add<bool>("issync", 'i', "need synchronization", false, true);
  a.parse_check(argc, argv);
  need_sync=a.get<bool>("issync");
  std::string img_topic = a.get<std::string>("camera_topic1");
  std::string img_topic2 = a.get<std::string>("camera_topic2");
  std::string save_path = a.get<std::string>("path");
  img_save_path=save_path+"/img1";
  img_save_path2=save_path+"/img2";
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
  if ((dir=opendir(img_save_path2.c_str())) == NULL)   
  { 
    std::string cmdpath ="mkdir -p "+img_save_path2;
    system(cmdpath.c_str());
  }
  if(need_sync){
    ROS_WARN("Starting recording with synchronization...");
  }else{
    ROS_WARN("Starting recording without synchronization...");
  }

  // 建立需要订阅的消息对应的订阅器
  ros::Subscriber img_sub;
  ros::Subscriber img_sub2;
  message_filters::Subscriber<Image> ImageInfo_sub;
  message_filters::Subscriber<Image> Image2Info_sub;
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ImageInfo_sub,
                                  Image2Info_sub);  // queue size=10
  sync.registerCallback(boost::bind(&Syncallback, _1, _2));
  if (need_sync) {
    ImageInfo_sub.subscribe(nh, img_topic, 1);
    Image2Info_sub.subscribe(nh, img_topic2, 1);
  } else {
    img_sub = nh.subscribe<sensor_msgs::Image>(img_topic, 10, Imgcallback);
    img_sub2 = nh.subscribe<sensor_msgs::Image>(img_topic2, 10, Imgcallback2);
  }

  thread t(keyaction);
  t.detach();
  ros::spin();
  return 0;
}