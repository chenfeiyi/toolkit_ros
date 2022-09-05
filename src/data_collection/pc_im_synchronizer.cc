#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>

#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <string>
#include <vector>
#include <dirent.h>

#include "ros/ros.h"
#include "../common/cmdline.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

string pcd_save_path ="";
string img_save_path ="";
// /*********这个是针对和赛雷达定义的点云类型**********/
// struct MyPointType {
//   PCL_ADD_POINT4D;  // 添加pcl里xyz+padding
//   uint8_t intensity;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保定义新类型点云内存与SSE对齐
// } EIGEN_ALIGN16;                   // 强制SSE填充以正确对齐内存
// POINT_CLOUD_REGISTER_POINT_STRUCT(
//     MyPointType,  // 定义新类型里元素包括XYZ+“test”
//     (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity))

void Syncallback(const PointCloud2ConstPtr& ori_pointcloud,
                 const ImageConstPtr& ori_image) {
  cout << "\033[1;32m Syn! \033[0m" << endl;
  PointCloud2 syn_pointcloud;
  Image syn_iamge;
  syn_pointcloud = *ori_pointcloud;
  syn_iamge = *ori_image;
  cout << "syn pointcloud' timestamp : " << syn_pointcloud.header.stamp << endl;
  cout << "syn image's timestamp : " << syn_iamge.header.stamp << endl;

  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  cv_bridge::CvImagePtr cv_image_ptr;

  cv::Mat cv_image;

  cv_image_ptr =
      cv_bridge::toCvCopy(ori_image, sensor_msgs::image_encodings::BGR8);

  cv_image = cv_image_ptr->image;

  pcl::fromROSMsg(syn_pointcloud, pcl_cloud);

  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // 选择jpeg
  compression_params.push_back(100);  // 在这个填入你要的图片质量
  cv::imwrite(img_save_path + std::to_string(ori_image->header.stamp.toSec()) + ".jpg",
              cv_image, compression_params);
  pcl::io::savePCDFile(
      pcd_save_path + std::to_string(ori_pointcloud->header.stamp.toSec()) + ".pcd",
      pcl_cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "synchronizer");
  ros::NodeHandle node;
  cmdline::parser a;
  a.add<std::string>("cam_topic", 'c', "camera topic name", false, "/usb_cam/image_raw");
  a.add<std::string>("lidar_topic", 'l', "lidar topic name", false, "/velodyne_points");
  a.add<std::string>("path", 'p', "save path", true);
  a.parse_check(argc, argv);
  std::string pcd_topic = a.get<std::string>("lidar_topic");
  std::string img_topic = a.get<std::string>("cam_topic");
  std::string save_path = a.get<std::string>("path");
  img_save_path=save_path+"/img/";
  pcd_save_path=save_path+"/pcd/";
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
  if ((dir=opendir(pcd_save_path.c_str())) == NULL)   
  { 
    std::string cmdpath ="mkdir -p "+pcd_save_path;
    system(cmdpath.c_str());
  }
  // ******* 记载参数 ********
  ROS_WARN("synchronizer");

  // 建立需要订阅的消息对应的订阅器
  message_filters::Subscriber<PointCloud2> PointCloudInfo_sub(node,pcd_topic, 1);
  message_filters::Subscriber<Image> ImageInfo_sub(node, img_topic, 1);

  typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PointCloudInfo_sub,
                                  ImageInfo_sub);  // queue size=10
  sync.registerCallback(boost::bind(&Syncallback, _1, _2));

  ros::spin();
  return 0;
}