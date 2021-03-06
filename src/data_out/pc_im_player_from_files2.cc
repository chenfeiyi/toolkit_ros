/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-13 19:41:51
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
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_common.h>

#include <opencv2/opencv.hpp>
#include <string>

#include "file_operation.hpp"
#include "cmdline.h"
using namespace std;

/** 
 * @brief: play image and point cloud at the same time from files.
 * It requires that the point cloud and image files form one-to-one correspondence.
 * And the file name should the sequence number or the timestamp.
 * What's more it adds camera information manager to meet some needs
 */

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "data_player");
  ros::NodeHandle nh;

  cmdline::parser a;
  a.add<bool>("loop", 'l', "play data repeatly", false, false);
  a.add<float>("hz", 'z', "frequency to paly the data", false, 10);
  a.parse_check(argc, argv);
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher trans_cam_pub =
        it.advertiseCamera("/usb_cam/image_raw", 10);
    ros::Publisher img_pub =
        nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 10);
    ros::Publisher pcd_pub =
        nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
    std::string img_file_path =
        "/media/ramlab/SolidDisk/vel2cam2021/seq/02/img/";
    std::string pcd_file_path =
        "/media/ramlab/SolidDisk/vel2cam2021/seq/02/pcd/";

    vector<string> img_file_list;
    vector<string> pcd_file_list;
    calibrator_pipeline::common::GetFilelist(img_file_path, &img_file_list,
                                             false);
    calibrator_pipeline::common::GetFilelist(pcd_file_path, &pcd_file_list,
                                             false);

    sort(img_file_list.begin(), img_file_list.end());
    sort(pcd_file_list.begin(), pcd_file_list.end());

    ros::Rate rate(a.get<float>("hz"));
    sensor_msgs::ImagePtr img_msg;
    sensor_msgs::PointCloud2 pcd_msg;
    camera_info_manager::CameraInfoManager cam_info_manager(
        nh, "usb_cam", "file:///home/ramlab/.ros/camera_info2/head_camera.yaml");
    sensor_msgs::CameraInfoPtr ci(
        new sensor_msgs::CameraInfo(cam_info_manager.getCameraInfo()));

    std::cout << "******************************" << std::endl;
    std::cout << "start to play data" << std::endl;
    int i = 0;
    while (ros::ok()) {
      std::string img_name = img_file_path + img_file_list[2*i];
      std::string pcd_name = pcd_file_path + pcd_file_list[i];
      cv::Mat img = cv::imread(img_name);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::io::loadPCDFile(pcd_name, *cloud_raw);

      pcl::toROSMsg(*cloud_raw, pcd_msg);

      std::vector<std::string> img_field;
      std::vector<std::string> pcd_field;
      calibrator_pipeline::common::SplitString(img_file_list[2*i],&img_field,'.');
      calibrator_pipeline::common::SplitString(pcd_file_list[i],&pcd_field,'.');
      if (pcd_field.size() != 3 || img_field.size() != 3) {
        std::cout << "Wrong file name1: " << img_file_list[2*i] << std::endl;
        std::cout << "Wrong file name2: " << pcd_file_list[i] << std::endl;
        continue;
      }

      pcd_msg.header.frame_id = "velodyne";
      pcd_msg.header.stamp =
          ros::Time(std::atof((pcd_field[0] + '.' + pcd_field[1]).c_str()));
      img_msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      img_msg->header.frame_id = "usb_cam";
      img_msg->header.stamp =
          ros::Time(std::atof((img_field[0] + '.' + img_field[1]).c_str()));
      ci->header.frame_id = img_msg->header.frame_id;
      ci->header.stamp = img_msg->header.stamp;

    //   img_pub.publish(img_msg);
      trans_cam_pub.publish(img_msg, ci);
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
